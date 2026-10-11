use embassy_futures::select::{select, select3};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal::digital::InputPin;
use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::digital::Wait;
use embedded_hal_async::spi::SpiDevice;
use xarxa_driver::{Timestamp, TxTimestamp};
use xarxa_dsa::{PortEvent, PortId, PortLink, StpState};

use super::state::{
    DiagError, DiagRequest, MAX_PORTS, RegWidth, SwitchState, T1_LINK, TX_TIMESTAMP_TIMEOUT, link_from,
};
use crate::fmt::{debug, info, warning};
use crate::ptp::{self, TxClass};
use crate::t1::{Role, T1Init};
use crate::{Error, Lan937x, Port, mib, reg};

const PHC_REFRESH: Duration = Duration::from_secs(1);
/// Interrupts carry link changes; this catches any that are missed.
const LINK_POLL: Duration = Duration::from_secs(2);
/// One port's counters are read per tick, well before any can wrap.
const MIB_TICK: Duration = Duration::from_millis(500);

/// The task that services a LAN937x. See the [module documentation](super).
pub struct Runner<'a, SPI, D, IRQ> {
    dev: Lan937x<SPI, D>,
    irq: IRQ,
    state: &'a SwitchState,
    /// T1 PHY initializations in progress.
    t1_init: [Option<T1Init>; MAX_PORTS],
    /// Ports with an internal T1 PHY.
    t1_ports: u8,
    /// Port interrupts are enabled once the T1 PHYs are initialized.
    interrupts_enabled: bool,
    next_phc: Instant,
    next_link_poll: Instant,
    next_mib: Instant,
    mib_port: u8,
    /// T1 ports whose PHY is in loopback. They don't learn: everything that
    /// comes back in on them was sent by someone else.
    looped: u8,
    /// IRQ_N stayed asserted after servicing: a source this runner doesn't
    /// clear. Don't spin on it.
    irq_stuck: bool,
}

impl<'a, SPI, D, IRQ> Runner<'a, SPI, D, IRQ>
where
    SPI: SpiDevice,
    D: DelayNs,
    IRQ: Wait + InputPin,
{
    /// Service `dev`, which [`Lan937x::init`](crate::Lan937x::init) has
    /// configured to match `state`. `irq` is the switch's IRQ_N line (active
    /// low). `t1_ports` are the ports with an internal T1 PHY and their initial
    /// roles; the runner initializes those PHYs first.
    pub fn new(dev: Lan937x<SPI, D>, irq: IRQ, state: &'a SwitchState, t1_ports: &[(Port, Role)]) -> Self {
        let mut t1_mask = 0;
        state.lock(|inner| {
            for &(port, role) in t1_ports {
                inner.ports[port.0 as usize].role = Some(role);
                inner.ports[port.0 as usize].reinit = true;
                t1_mask |= 1 << port.0;
            }
        });
        let now = Instant::now();
        Self {
            dev,
            irq,
            state,
            t1_init: [const { None }; MAX_PORTS],
            t1_ports: t1_mask,
            interrupts_enabled: false,
            next_phc: now,
            next_link_poll: now + LINK_POLL,
            next_mib: now + MIB_TICK,
            mib_port: 0,
            looped: 0,
            irq_stuck: false,
        }
    }

    /// Run forever.
    pub async fn run(mut self) -> ! {
        loop {
            if let Err(e) = self.service().await {
                warning!("lan937x: switch access failed: {}", e.kind());
                Timer::after_millis(10).await;
            }
            if self.t1_init_pending() {
                // Initialization runs a step at a time; let other tasks in between.
                embassy_futures::yield_now().await;
                continue;
            }
            let deadline = self.next_deadline();
            let stuck = self.irq.is_low().unwrap_or(false);
            if stuck && !self.irq_stuck {
                warning!("lan937x: IRQ_N stays asserted; polling it every 10 ms");
            }
            self.irq_stuck = stuck;
            if stuck {
                select(
                    self.state.work.wait(),
                    Timer::at(deadline.min(Instant::now() + Duration::from_millis(10))),
                )
                .await;
            } else {
                select3(self.state.work.wait(), self.irq.wait_for_low(), Timer::at(deadline)).await;
            }
        }
    }

    /// One round of work, in priority order (design §7.3): spanning-tree
    /// states and flushes, then PTP, then link changes, then the rest.
    async fn service(&mut self) -> Result<(), Error<SPI::Error>> {
        self.apply_stp().await?;
        self.apply_flushes().await?;
        if let Some(request) = self.state.diag_request.try_take() {
            let result = self.run_diag(request).await;
            self.state.diag_response.signal(result);
        }
        if self.irq.is_low().unwrap_or(false) {
            self.service_interrupts().await?;
        }
        self.apply_phc_requests().await?;
        self.apply_external_links().await?;
        self.apply_loopbacks().await?;
        self.apply_led_modes().await?;
        self.t1_init_step().await?;

        let now = Instant::now();
        if now >= self.next_phc {
            self.next_phc = now + PHC_REFRESH;
            self.refresh_phc().await?;
        }
        self.expire_timestamps();
        if now >= self.next_link_poll {
            self.next_link_poll = now + LINK_POLL;
            self.poll_t1_links().await?;
        }
        if now >= self.next_mib {
            self.next_mib = now + MIB_TICK;
            self.read_counters().await?;
        }
        Ok(())
    }

    fn next_deadline(&self) -> Instant {
        let oldest_timestamp = self.state.lock(|inner| {
            inner
                .pending_timestamps
                .iter()
                .flatten()
                .flatten()
                .map(|p| p.since + TX_TIMESTAMP_TIMEOUT)
                .min()
        });
        let mut deadline = self.next_phc.min(self.next_link_poll).min(self.next_mib);
        if let Some(t) = oldest_timestamp {
            deadline = deadline.min(t);
        }
        deadline
    }

    /// Apply requested spanning-tree states, one port at a time, until none
    /// is outstanding. Closing a port writes its state before the membership
    /// masks, opening one after them, so no frame is forwarded through a port
    /// that is being closed.
    async fn apply_stp(&mut self) -> Result<(), Error<SPI::Error>> {
        let config = &self.state.config;
        loop {
            let next = self.state.lock(|inner| {
                let mut forwarding = 0u8;
                let mut next = None;
                for i in 0..config.ports as usize {
                    if i == config.host.0 as usize {
                        continue;
                    }
                    let view = inner.ports[i].stp;
                    if view.applied == StpState::Forwarding {
                        forwarding |= 1 << i;
                    }
                    // An administratively disabled port is disabled in hardware.
                    let target = if inner.ports[i].enabled {
                        view.requested
                    } else {
                        StpState::Disabled
                    };
                    if next.is_none() && target != view.applied {
                        next = Some((i, target, view.applied));
                    }
                }
                next.map(|n| (n, forwarding))
            });
            let Some(((i, target, applied), mut forwarding)) = next else {
                return Ok(());
            };
            let port = Port(i as u8);
            if target == StpState::Forwarding {
                forwarding |= 1 << i;
            } else {
                forwarding &= !(1 << i);
            }
            if target < applied {
                self.dev.write_stp_state(port, target.into()).await?;
                self.dev.write_membership(config.ports, config.host, forwarding).await?;
            } else {
                self.dev.write_membership(config.ports, config.host, forwarding).await?;
                self.dev.write_stp_state(port, target.into()).await?;
            }
            if self.looped & (1 << i) != 0 {
                self.dev.set_learning(port, false).await?;
            }
            self.state.lock(|inner| inner.ports[i].stp.applied = target);
            debug!("lan937x: port {} now {}", port.datasheet_number(), target);
            self.state.publish(PortEvent::StpApplied {
                port: PortId(i as u8),
                state: target,
            });
        }
    }

    async fn apply_flushes(&mut self) -> Result<(), Error<SPI::Error>> {
        for i in 0..self.state.config.ports as usize {
            let requested = self
                .state
                .lock(|inner| core::mem::take(&mut inner.ports[i].flush_requested));
            if !requested {
                continue;
            }
            self.dev.flush_port(Port(i as u8)).await?;
            let done = self.state.lock(|inner| {
                let port = &mut inner.ports[i];
                // A flush requested meanwhile is still pending.
                port.flush_pending = port.flush_requested;
                !port.flush_pending
            });
            if done {
                self.state.publish(PortEvent::FlushDone { port: PortId(i as u8) });
            }
        }
        Ok(())
    }

    async fn service_interrupts(&mut self) -> Result<(), Error<SPI::Error>> {
        let pending = self.dev.pending_ports().await?;
        for i in 0..self.state.config.ports {
            if pending & (1 << i) == 0 {
                continue;
            }
            let port = Port(i);
            let sources = self.dev.port_interrupts(port).await?;
            if sources & reg::P_INT_PTP != 0 {
                self.collect_timestamps(port).await?;
            }
            if sources & reg::P_INT_PHY != 0 {
                self.dev.t1_ack_interrupts(port).await?;
                self.update_t1_link(port).await?;
            }
        }
        Ok(())
    }

    /// Read `port`'s transmit timestamps and hand them to the stack.
    async fn collect_timestamps(&mut self, port: Port) -> Result<(), Error<SPI::Error>> {
        let raws = self.dev.ptp_take_tx_timestamps(port).await?;
        let Some(now) = self.state.phc_now() else {
            return Ok(());
        };
        let i = port.0 as usize;
        self.state.lock(|inner| {
            for class in TxClass::ALL {
                let Some(raw) = raws[class.index()] else { continue };
                let Some(pending) = inner.pending_timestamps[i][class.index()].take() else {
                    continue;
                };
                let time = ptp::reconstruct(raw, now);
                if inner.tx_timestamps.is_full() {
                    inner.tx_timestamps.pop_front();
                }
                let _ = inner.tx_timestamps.push_back(TxTimestamp {
                    id: pending.id,
                    timestamp: Timestamp::from_seconds_and_nanos(time.sec, time.ns),
                });
            }
        });
        self.state.wake_stack();
        Ok(())
    }

    fn expire_timestamps(&mut self) {
        self.state.lock(|inner| {
            for slot in inner.pending_timestamps.iter_mut().flatten() {
                if slot.is_some_and(|p| p.since.elapsed() > TX_TIMESTAMP_TIMEOUT) {
                    *slot = None;
                }
            }
        });
    }

    async fn refresh_phc(&mut self) -> Result<(), Error<SPI::Error>> {
        let time = self.dev.ptp_read().await?;
        self.state
            .lock(|inner| inner.phc_reference = Some((time, Instant::now())));
        Ok(())
    }

    async fn apply_phc_requests(&mut self) -> Result<(), Error<SPI::Error>> {
        let (step, rate) = self.state.lock(|inner| (inner.phc_step.take(), inner.phc_rate.take()));
        if let Some(delta) = step {
            self.dev.ptp_step(delta).await?;
            // Timestamps must be rebuilt from a reading after the step.
            self.refresh_phc().await?;
        }
        if let Some(scaled_ppm) = rate {
            self.dev.ptp_adjust_rate(scaled_ppm).await?;
        }
        Ok(())
    }

    async fn apply_external_links(&mut self) -> Result<(), Error<SPI::Error>> {
        for i in 0..self.state.config.ports as usize {
            let Some(link) = self.state.lock(|inner| inner.external_links[i].take()) else {
                continue;
            };
            let port = Port(i as u8);
            if link.up {
                // The MAC must match the PHY before traffic flows.
                self.dev
                    .set_xmii_link(port, link.speed, link.duplex, link.tx_pause, link.rx_pause)
                    .await?;
            }
            self.set_link(port, link_from(&link));
        }
        Ok(())
    }

    async fn apply_loopbacks(&mut self) -> Result<(), Error<SPI::Error>> {
        for i in 0..MAX_PORTS {
            if self.t1_ports & (1 << i) == 0 || self.t1_init[i].is_some() {
                continue;
            }
            let Some(on) = self.state.lock(|inner| inner.ports[i].loopback.take()) else {
                continue;
            };
            let port = Port(i as u8);
            self.dev.t1_set_loopback(port, on).await?;
            if on {
                self.looped |= 1 << i;
                self.dev.set_learning(port, false).await?;
                self.dev.flush_port(port).await?;
            } else {
                self.looped &= !(1 << i);
                let applied = self.state.lock(|inner| inner.ports[i].stp.applied);
                self.dev.write_stp_state(port, applied.into()).await?;
            }
            info!("lan937x: port {} loopback {}", i + 1, on);
        }
        Ok(())
    }

    async fn apply_led_modes(&mut self) -> Result<(), Error<SPI::Error>> {
        // Not while a PHY initializes: its reset reverts the mode anyway.
        let ready = self.t1_ports
            & !self
                .t1_init
                .iter()
                .enumerate()
                .fold(0u8, |m, (i, p)| m | ((p.is_some() as u8) << i));
        let (mode, pending) = self.state.lock(|inner| {
            let pending = inner.led_pending & ready;
            inner.led_pending &= !pending;
            (inner.led_mode, pending)
        });
        for i in 0..MAX_PORTS {
            if pending & (1 << i) != 0 {
                self.dev.t1_set_led_mode(Port(i as u8), mode).await?;
            }
        }
        Ok(())
    }

    fn set_link(&self, port: Port, link: PortLink) {
        let changed = self.state.lock(|inner| {
            let state = &mut inner.ports[port.0 as usize].link;
            let changed = *state != link;
            *state = link;
            changed
        });
        if !changed {
            return;
        }
        info!(
            "lan937x: port {} link {}",
            port.datasheet_number(),
            if link.up { "up" } else { "down" }
        );
        let id = PortId(port.0);
        self.state.publish(if link.up {
            PortEvent::LinkUp {
                port: id,
                speed: link.speed,
                duplex: link.duplex,
            }
        } else {
            PortEvent::LinkDown { port: id }
        });
    }

    async fn update_t1_link(&mut self, port: Port) -> Result<(), Error<SPI::Error>> {
        let up = self.dev.t1_link(port).await?;
        self.set_link(port, if up { T1_LINK } else { PortLink::DOWN });
        Ok(())
    }

    async fn poll_t1_links(&mut self) -> Result<(), Error<SPI::Error>> {
        for i in 0..MAX_PORTS {
            if self.t1_ports & (1 << i) != 0 && self.t1_init[i].is_none() {
                self.update_t1_link(Port(i as u8)).await?;
            }
        }
        Ok(())
    }

    fn t1_init_pending(&self) -> bool {
        self.t1_init.iter().any(Option::is_some) || self.state.lock(|inner| inner.ports.iter().any(|p| p.reinit))
    }

    /// One step of a T1 PHY initialization, if any is due.
    async fn t1_init_step(&mut self) -> Result<(), Error<SPI::Error>> {
        // Start the requested (re)initializations.
        for i in 0..MAX_PORTS {
            let role = self.state.lock(|inner| {
                let port = &mut inner.ports[i];
                core::mem::take(&mut port.reinit).then(|| port.role.unwrap_or(Role::Slave))
            });
            if let Some(role) = role {
                self.t1_init[i] = Some(T1Init::new(role));
                // The link drops while the PHY reinitializes.
                self.set_link(Port(i as u8), PortLink::DOWN);
            }
        }
        let Some(i) = self.t1_init.iter().position(Option::is_some) else {
            return Ok(());
        };
        let port = Port(i as u8);
        let init = self.t1_init[i].as_mut().unwrap();
        let result = self.dev.t1_init_step(port, init).await;
        let init = self.t1_init[i].as_ref().unwrap();
        let done = match result {
            Ok(done) => done,
            Err(e) => {
                // Start over: a step that failed half way leaves the PHY unknown.
                warning!(
                    "lan937x: port {} PHY init failed at step {} ({}), restarting",
                    port.datasheet_number(),
                    init.steps(),
                    e.kind()
                );
                self.state.lock(|inner| inner.ports[i].reinit = true);
                self.t1_init[i] = None;
                return Err(e);
            }
        };
        if done {
            self.t1_init[i] = None;
            debug!("lan937x: port {} PHY initialized", port.datasheet_number());
            self.state.lock(|inner| inner.led_pending |= 1 << i);
            self.update_t1_link(port).await?;
            if !self.interrupts_enabled && self.t1_init.iter().all(Option::is_none) {
                self.enable_interrupts().await?;
            }
        }
        Ok(())
    }

    async fn enable_interrupts(&mut self) -> Result<(), Error<SPI::Error>> {
        let config = &self.state.config;
        let user_ports = ((1u16 << config.ports) - 1) as u8 & !(1 << config.host.0);
        for i in 0..config.ports {
            if user_ports & (1 << i) != 0 {
                self.dev.ptp_enable_tx_interrupts(Port(i)).await?;
            }
        }
        self.dev
            .enable_port_interrupts(user_ports & !self.t1_ports, reg::P_INT_PTP)
            .await?;
        self.dev
            .enable_port_interrupts(self.t1_ports, reg::P_INT_PTP | reg::P_INT_PHY)
            .await?;
        self.interrupts_enabled = true;
        info!("lan937x: interrupts enabled");
        Ok(())
    }

    /// Read one port's counters into the totals; they clear when read.
    async fn read_counters(&mut self) -> Result<(), Error<SPI::Error>> {
        let port = Port(self.mib_port);
        self.mib_port = (self.mib_port + 1) % self.state.config.ports;
        let mut c = [0u64; 11];
        for (value, index) in c.iter_mut().zip([
            mib::RX_BYTES,
            mib::TX_BYTES,
            mib::RX_UNICAST,
            mib::RX_MULTICAST,
            mib::RX_BROADCAST,
            mib::TX_UNICAST,
            mib::TX_MULTICAST,
            mib::TX_BROADCAST,
            mib::RX_DROPS,
            mib::TX_DROPS,
            mib::RX_CRC_ERROR,
        ]) {
            *value = self.dev.read_mib(port, index).await?;
        }
        self.state.lock(|inner| {
            let t = &mut inner.ports[port.0 as usize].counters;
            t.rx_bytes += c[0];
            t.tx_bytes += c[1];
            t.rx_unicast += c[2];
            t.rx_multicast += c[3];
            t.rx_broadcast += c[4];
            t.tx_unicast += c[5];
            t.tx_multicast += c[6];
            t.tx_broadcast += c[7];
            t.rx_drops += c[8];
            t.tx_drops += c[9];
            t.rx_crc_errors += c[10];
        });
        Ok(())
    }

    async fn run_diag(&mut self, request: DiagRequest) -> Result<u64, DiagError> {
        let t1_ports = self.t1_ports;
        let t1 = |port: Port| t1_ports & (1 << port.0) != 0;
        let result = match request {
            DiagRequest::ReadReg { addr, width } => match width {
                RegWidth::U8 => self.dev.regs().read8(addr).await.map(u64::from),
                RegWidth::U16 => self.dev.regs().read16(addr).await.map(u64::from),
                RegWidth::U32 => self.dev.regs().read32(addr).await.map(u64::from),
            },
            DiagRequest::WriteReg { addr, width, value } => match width {
                RegWidth::U8 => self.dev.regs().write8(addr, value as u8).await.map(|_| 0),
                RegWidth::U16 => self.dev.regs().write16(addr, value as u16).await.map(|_| 0),
                RegWidth::U32 => self.dev.regs().write32(addr, value).await.map(|_| 0),
            },
            DiagRequest::ReadPhy { port, bank, reg } => {
                if !t1(port) {
                    return Err(DiagError::NotT1Port);
                }
                self.dev.t1_read(port, bank, reg).await.map(u64::from)
            }
            DiagRequest::WritePhy { port, bank, reg, value } => {
                if !t1(port) {
                    return Err(DiagError::NotT1Port);
                }
                self.dev.t1_write(port, bank, reg, value).await.map(|_| 0)
            }
            DiagRequest::ReadMib { port, index } => self.dev.read_mib(port, index).await,
            DiagRequest::Sqi { port } => {
                if !t1(port) {
                    return Err(DiagError::NotT1Port);
                }
                self.dev.t1_sqi(port).await.map(u64::from)
            }
        };
        result.map_err(|e| match e {
            Error::Timeout => DiagError::Timeout,
            _ => DiagError::Bus,
        })
    }

    /// Direct access to the switch, for diagnostics. Don't change state the
    /// runner manages.
    pub fn device(&mut self) -> &mut Lan937x<SPI, D> {
        &mut self.dev
    }
}
