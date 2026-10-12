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

/// Rounds in a row that may leave IRQ_N asserted before it counts as stuck.
const MAX_IRQ_ROUNDS: u8 = 8;
const PHC_REFRESH: Duration = Duration::from_secs(1);
/// Interrupts carry link changes; this catches any that are missed.
const LINK_POLL: Duration = Duration::from_secs(2);
/// One port's counters are read per tick, well before any can wrap.
const MIB_TICK: Duration = Duration::from_millis(500);
/// The pause before a round that failed is tried again; it doubles, up to
/// [`RETRY_MAX`], while rounds keep failing.
const RETRY_FIRST: Duration = Duration::from_millis(10);
const RETRY_MAX: Duration = Duration::from_millis(500);

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
    /// Each user port's spanning-tree state in the hardware (learning off if
    /// it's looped), as far as the runner knows: `None` before it's written
    /// and after a write that failed, so it's written again.
    hw_stp: [Option<StpState>; MAX_PORTS],
    /// The forwarding ports the hardware's membership masks are for, or
    /// `None` before they're written and after a write that failed.
    hw_membership: Option<u8>,
    /// IRQ_N stayed asserted after servicing: a source this runner doesn't
    /// clear. Don't spin on it.
    irq_stuck: bool,
    /// Rounds failed in a row, and the pause before the next try.
    failures: u32,
    retry: Duration,
    /// What the last round's interrupt service found: `None` if it didn't
    /// run, else whether any port had an interrupt pending.
    irq_found: Option<bool>,
    /// Rounds in a row that left IRQ_N asserted.
    irq_rounds: u8,
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
            hw_stp: [None; MAX_PORTS],
            hw_membership: None,
            irq_stuck: false,
            failures: 0,
            retry: RETRY_FIRST,
            irq_found: None,
            irq_rounds: 0,
        }
    }

    /// Run forever.
    pub async fn run(mut self) -> ! {
        loop {
            self.state.count_round();
            match self.service().await {
                Ok(()) if self.failures != 0 => {
                    info!(
                        "lan937x: switch access works again after {} failed rounds",
                        self.failures
                    );
                    self.failures = 0;
                    self.retry = RETRY_FIRST;
                }
                Ok(()) => {}
                Err(e) => {
                    // What failed is still outstanding: try again soon, less
                    // often while it keeps failing. Warn once per run of
                    // failures.
                    if self.failures == 0 {
                        warning!("lan937x: switch access failed: {}; retrying", e.kind());
                    }
                    self.failures = self.failures.saturating_add(1);
                    Timer::after(self.retry).await;
                    self.retry = (self.retry * 2).min(RETRY_MAX);
                    continue;
                }
            }
            if self.t1_init_pending() {
                // Initialization runs a step at a time; let other tasks in between.
                embassy_futures::yield_now().await;
                continue;
            }
            let deadline = self.next_deadline();
            let low = self.irq.is_low().unwrap_or(false);
            // Asserted after a round that cleared something, or didn't look:
            // a new interrupt, so service it now. Stuck only if a round found
            // nothing to clear and it's still asserted.
            // A source that clearing doesn't release would keep this busy,
            // though: after a few rounds it counts as stuck too.
            self.irq_rounds = if low { self.irq_rounds.saturating_add(1) } else { 0 };
            if low && self.irq_found != Some(false) && self.irq_rounds <= MAX_IRQ_ROUNDS {
                continue;
            }
            let stuck = low;
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
    ///
    /// Work that fails stays outstanding for the next round, and doesn't stop
    /// the rest of this one (one failing PHY mustn't hold up the switch).
    /// Returns the first error.
    async fn service(&mut self) -> Result<(), Error<SPI::Error>> {
        let mut result = self.apply_stp().await;
        result = result.and(self.apply_flushes().await);
        if let Some(request) = self.state.diag_request.try_take() {
            let response = self.run_diag(request).await;
            self.state.diag_response.signal(response);
        }
        self.irq_found = None;
        if self.irq.is_low().unwrap_or(false) {
            result = result.and(self.service_interrupts().await);
        }
        result = result.and(self.apply_phc_requests().await);
        result = result.and(self.apply_external_links().await);
        result = result.and(self.apply_power().await);
        result = result.and(self.apply_loopbacks().await);
        result = result.and(self.apply_led_modes().await);
        result = result.and(self.t1_init_step().await);

        let now = Instant::now();
        if now >= self.next_phc {
            self.next_phc = now + PHC_REFRESH;
            result = result.and(self.refresh_phc().await);
        }
        self.expire_timestamps();
        if now >= self.next_link_poll {
            self.next_link_poll = now + LINK_POLL;
            result = result.and(self.poll_t1_links().await);
        }
        if now >= self.next_mib {
            self.next_mib = now + MIB_TICK;
            result = result.and(self.read_counters().await);
        }
        result
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

    /// Apply requested spanning-tree states, one port at a time, until the
    /// hardware has them all. Closing a port writes its state before the
    /// membership masks, opening one after them, so no frame is forwarded
    /// through a port that is being closed.
    ///
    /// Opening is reported before the writes and closing after them, so the
    /// applied state never understates the hardware (the `SwitchPorts`
    /// contract spanning tree relies on). A request that arrives meanwhile is
    /// applied on the next pass. What's written is compared with what the
    /// hardware has, not with the applied state: an opening whose writes
    /// failed after it was reported is still outstanding, and is written
    /// again on the next round.
    async fn apply_stp(&mut self) -> Result<(), Error<SPI::Error>> {
        let state = self.state;
        let config = &state.config;
        loop {
            let (next, mut forwarding) = state.lock(|inner| {
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
                    if next.is_none() && (target != view.applied || self.hw_stp[i] != Some(target)) {
                        next = Some((i, target, view.applied));
                    }
                }
                (next, forwarding)
            });
            let Some((i, target, applied)) = next else {
                // Every state is in place; the masks may not be, after a failure.
                return self.write_membership(forwarding).await;
            };
            let port = Port(i as u8);
            if target == StpState::Forwarding {
                forwarding |= 1 << i;
            } else {
                forwarding &= !(1 << i);
            }
            let report = || {
                state.lock(|inner| inner.ports[i].stp.applied = target);
                debug!("lan937x: port {} now {}", port.datasheet_number(), target);
                state.publish(PortEvent::StpApplied {
                    port: PortId(i as u8),
                    state: target,
                });
            };
            if target < applied {
                self.write_stp(port, target).await?;
                self.write_membership(forwarding).await?;
                report();
            } else {
                // Opening, or writing again what a failure left unknown.
                if target != applied {
                    report();
                }
                self.write_membership(forwarding).await?;
                self.write_stp(port, target).await?;
            }
        }
    }

    /// Write `port`'s spanning-tree state, unless the hardware has it.
    async fn write_stp(&mut self, port: Port, state: StpState) -> Result<(), Error<SPI::Error>> {
        let i = port.0 as usize;
        if self.hw_stp[i] == Some(state) {
            return Ok(());
        }
        self.hw_stp[i] = None;
        self.dev.write_stp_state(port, state.into()).await?;
        if self.looped & (1 << i) != 0 {
            self.dev.set_learning(port, false).await?;
        }
        self.hw_stp[i] = Some(state);
        Ok(())
    }

    /// Write the membership masks for the forwarding user ports `forwarding`,
    /// unless the hardware has them.
    async fn write_membership(&mut self, forwarding: u8) -> Result<(), Error<SPI::Error>> {
        if self.hw_membership == Some(forwarding) {
            return Ok(());
        }
        let config = &self.state.config;
        self.hw_membership = None;
        self.dev.write_membership(config.ports, config.host, forwarding).await?;
        self.hw_membership = Some(forwarding);
        Ok(())
    }

    async fn apply_flushes(&mut self) -> Result<(), Error<SPI::Error>> {
        let mut result = Ok(());
        for i in 0..self.state.config.ports as usize {
            let requested = self
                .state
                .lock(|inner| core::mem::take(&mut inner.ports[i].flush_requested));
            if !requested {
                continue;
            }
            if let Err(e) = self.dev.flush_port(Port(i as u8)).await {
                // Retried next round. It may have left learning off: the
                // spanning-tree state is written again first.
                self.state.lock(|inner| inner.ports[i].flush_requested = true);
                self.hw_stp[i] = None;
                result = result.and(Err(e));
                continue;
            }
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
        result
    }

    async fn service_interrupts(&mut self) -> Result<(), Error<SPI::Error>> {
        let pending = self.dev.pending_ports().await?;
        self.irq_found = Some(pending != 0);
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
                let (int, int2) = self.dev.t1_ack_interrupts(port).await?;
                let mode = self.dev.phy_read(port, crate::t1::MODE_STS).await?;
                debug!(
                    "lan937x: port {} PHY interrupt {:04x} {:04x} mode {:04x}",
                    port.datasheet_number(),
                    int,
                    int2,
                    mode
                );
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
        if let Some(delta) = self.state.lock(|inner| inner.phc_step.take()) {
            if let Err(e) = self.dev.ptp_step(delta).await {
                // Retried next round, with any step requested meanwhile.
                self.state
                    .lock(|inner| inner.phc_step = Some(inner.phc_step.unwrap_or(0) + delta));
                return Err(e);
            }
            // Timestamps must be rebuilt from a reading after the step.
            self.refresh_phc().await?;
        }
        if let Some(scaled_ppm) = self.state.lock(|inner| inner.phc_rate.take())
            && let Err(e) = self.dev.ptp_adjust_rate(scaled_ppm).await
        {
            // Retried next round, unless a newer rate replaced it.
            self.state.lock(|inner| {
                inner.phc_rate.get_or_insert(scaled_ppm);
            });
            return Err(e);
        }
        Ok(())
    }

    async fn apply_external_links(&mut self) -> Result<(), Error<SPI::Error>> {
        let mut result = Ok(());
        for i in 0..self.state.config.ports as usize {
            let Some(link) = self.state.lock(|inner| inner.external_links[i].take()) else {
                continue;
            };
            let port = Port(i as u8);
            if link.up {
                // The MAC must match the PHY before traffic flows.
                let programmed = self
                    .dev
                    .set_xmii_link(port, link.speed, link.duplex, link.tx_pause, link.rx_pause)
                    .await;
                if let Err(e) = programmed {
                    // Retried next round, unless a newer change replaced it.
                    self.state.lock(|inner| {
                        inner.external_links[i].get_or_insert(link);
                    });
                    result = result.and(Err(e));
                    continue;
                }
            }
            self.set_link(port, link_from(&link));
        }
        result
    }

    async fn apply_loopbacks(&mut self) -> Result<(), Error<SPI::Error>> {
        let mut result = Ok(());
        for i in 0..MAX_PORTS {
            if self.t1_ports & (1 << i) == 0 || self.t1_init[i].is_some() {
                continue;
            }
            let Some(on) = self.state.lock(|inner| inner.ports[i].loopback.take()) else {
                continue;
            };
            if let Err(e) = self.set_loopback(Port(i as u8), on).await {
                // Retried next round, unless a newer request replaced it. It
                // may have left learning wrong: the spanning-tree state is
                // written again first.
                self.state.lock(|inner| {
                    inner.ports[i].loopback.get_or_insert(on);
                });
                self.hw_stp[i] = None;
                result = result.and(Err(e));
            }
        }
        result
    }

    async fn set_loopback(&mut self, port: Port, on: bool) -> Result<(), Error<SPI::Error>> {
        let i = port.0 as usize;
        self.dev.t1_set_loopback(port, on).await?;
        if on {
            self.looped |= 1 << i;
            self.dev.set_learning(port, false).await?;
            self.dev.flush_port(port).await?;
        } else {
            self.looped &= !(1 << i);
            // Learning as the state has it. One that isn't known is written
            // by apply_stp.
            if let Some(state) = self.hw_stp[i] {
                self.dev.write_stp_state(port, state.into()).await?;
            }
        }
        info!("lan937x: port {} loopback {}", port.datasheet_number(), on);
        Ok(())
    }

    /// Power T1 PHYs down when their port is disabled, and up (and
    /// reinitialize them) when it's enabled again.
    async fn apply_power(&mut self) -> Result<(), Error<SPI::Error>> {
        let mut result = Ok(());
        for i in 0..MAX_PORTS {
            if self.t1_ports & (1 << i) == 0 {
                continue;
            }
            let Some(up) = self.state.lock(|inner| inner.ports[i].power.take()) else {
                continue;
            };
            let port = Port(i as u8);
            if !up {
                // An initialization in progress would power it up again.
                self.t1_init[i] = None;
            }
            if let Err(e) = self.dev.t1_set_power_down(port, !up).await {
                // Retried next round, unless the port was enabled or
                // disabled again meanwhile.
                self.state.lock(|inner| {
                    inner.ports[i].power.get_or_insert(up);
                });
                result = result.and(Err(e));
                continue;
            }
            if up {
                self.state.lock(|inner| inner.ports[i].reinit = true);
            } else {
                self.set_link(port, PortLink::DOWN);
            }
            info!(
                "lan937x: port {} PHY powered {}",
                port.datasheet_number(),
                if up { "up" } else { "down" }
            );
        }
        result
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
        let mut result = Ok(());
        for i in 0..MAX_PORTS {
            if pending & (1 << i) == 0 {
                continue;
            }
            if let Err(e) = self.dev.t1_set_led_mode(Port(i as u8), mode).await {
                // Retried next round.
                self.state.lock(|inner| inner.led_pending |= 1 << i);
                result = result.and(Err(e));
            }
        }
        result
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
            let enabled = self.state.lock(|inner| inner.ports[i].enabled);
            if self.t1_ports & (1 << i) != 0 && self.t1_init[i].is_none() && enabled {
                self.update_t1_link(Port(i as u8)).await?;
            }
        }
        Ok(())
    }

    fn t1_init_pending(&self) -> bool {
        // A disabled port's PHY stays powered down: its reinitialization waits.
        self.t1_init.iter().any(Option::is_some)
            || self
                .state
                .lock(|inner| inner.ports.iter().any(|p| p.reinit && p.enabled))
    }

    /// One step of a T1 PHY initialization, if any is due.
    async fn t1_init_step(&mut self) -> Result<(), Error<SPI::Error>> {
        // Start the requested (re)initializations.
        for i in 0..MAX_PORTS {
            let role = self.state.lock(|inner| {
                let port = &mut inner.ports[i];
                if !port.enabled {
                    return None;
                }
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
            // It stays in progress until the link is read and, after the last
            // PHY, interrupts are enabled, so a failure there is retried.
            self.update_t1_link(port).await?;
            let others = self.t1_init.iter().enumerate().any(|(j, p)| j != i && p.is_some());
            if !self.interrupts_enabled && !others {
                self.enable_interrupts().await?;
            }
            self.t1_init[i] = None;
            debug!("lan937x: port {} PHY initialized", port.datasheet_number());
            self.state.lock(|inner| inner.led_pending |= 1 << i);
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

#[cfg(test)]
mod tests;
