//! The runner against a model of the switch's registers whose writes can be
//! made to fail, as on a flaky SPI bus. Each test drives it a round at a time.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::convert::Infallible;
use std::rc::Rc;

use embedded_hal::spi::{ErrorKind, ErrorType};
use embedded_hal_async::spi::{Operation, SpiDevice};
use xarxa_dsa::{PortEvent, PortEvents, PortId, PortInfo, PortKind, StpState, SwitchPorts};

use super::Runner;
use crate::t1::{self, Role};
use crate::xarxa::{ExternalLink, StateConfig, SwitchState};
use crate::{Duplex, Error, Lan937x, Port, Speed, reg};

const HOST: Port = Port(5);
const UPLINK: Port = Port(4);
/// A port with a T1 PHY, in the tests that have one.
const T1: Port = Port(0);

static USER_PORTS: [PortInfo; 3] = [
    PortInfo {
        id: PortId(0),
        name: "lan3",
        kind: PortKind::Downstream,
    },
    PortInfo {
        id: PortId(1),
        name: "lan4",
        kind: PortKind::Downstream,
    },
    PortInfo {
        id: PortId(4),
        name: "uplink",
        kind: PortKind::Uplink,
    },
];

/// Fail writes to `addr`: let `skip` through, then fail `count`.
struct Fault {
    addr: u32,
    skip: usize,
    count: usize,
}

/// The switch's registers, by byte address, and its T1 PHYs' registers, by
/// indirect address ([`reg::t1_phy`]).
#[derive(Default)]
struct Regs {
    mem: BTreeMap<u32, u8>,
    phy: BTreeMap<u16, u16>,
    fault: Option<Fault>,
    /// Address table flushes that never finish.
    flush_stuck: usize,
}

impl Regs {
    fn fail_writes(&mut self, addr: u32, skip: usize, count: usize) {
        self.fault = Some(Fault { addr, skip, count });
    }

    fn get(&self, addr: u32, len: u32) -> u32 {
        (addr..addr + len).fold(0, |v, a| (v << 8) | *self.mem.get(&a).unwrap_or(&0) as u32)
    }

    fn set(&mut self, addr: u32, len: u32, value: u32) {
        for (n, a) in (addr..addr + len).rev().enumerate() {
            self.mem.insert(a, (value >> (8 * n)) as u8);
        }
    }

    fn fails(&mut self, addr: u32) -> bool {
        let Some(fault) = self.fault.as_mut().filter(|f| f.addr == addr) else {
            return false;
        };
        if fault.skip > 0 {
            fault.skip -= 1;
            return false;
        }
        fault.count -= 1;
        if fault.count == 0 {
            self.fault = None;
        }
        true
    }

    /// What the switch does after a write to `addr`.
    fn written(&mut self, addr: u32) {
        match addr {
            reg::GLB_SW_LUE_CTL1 if self.get(addr, 1) as u8 & reg::LUE_CTL1_FLUSH_MSTP != 0 => {
                if self.flush_stuck > 0 {
                    self.flush_stuck -= 1;
                } else {
                    self.set(addr, 1, self.get(addr, 1) & !reg::LUE_CTL1_FLUSH_MSTP as u32);
                }
            }
            reg::VPHY_IND_CTRL => self.indirect(),
            _ => {}
        }
    }

    /// An access through the indirect PHY window, which finishes at once.
    fn indirect(&mut self) {
        let ctrl = self.get(reg::VPHY_IND_CTRL, 2) as u16;
        if ctrl & reg::VPHY_IND_BUSY == 0 {
            return;
        }
        let addr = self.get(reg::VPHY_IND_ADDR, 2) as u16;
        let old = self.phy.get(&addr).copied().unwrap_or(0);
        if ctrl & reg::VPHY_IND_WRITE != 0 {
            let value = self.get(reg::VPHY_IND_DATA, 2) as u16;
            let value = match ((addr & 0xff) >> 2) as u8 {
                // Resets finish at once; power-down survives a soft reset.
                t1::BMCR if value & t1::BMCR_RESET != 0 => old & t1::BMCR_POWER_DOWN,
                t1::MDIO_CTL2 => value & !0x0080,
                _ => value,
            };
            self.phy.insert(addr, value);
        } else {
            self.set(reg::VPHY_IND_DATA, 2, old as u32);
        }
        self.set(reg::VPHY_IND_CTRL, 2, (ctrl & !reg::VPHY_IND_BUSY) as u32);
    }
}

/// A SPI device on [`Regs`].
struct Bus(Rc<RefCell<Regs>>);

impl ErrorType for Bus {
    type Error = ErrorKind;
}

impl SpiDevice for Bus {
    async fn transaction(&mut self, ops: &mut [Operation<'_, u8>]) -> Result<(), ErrorKind> {
        let mut regs = self.0.borrow_mut();
        let [Operation::Write(cmd), data] = ops else {
            panic!("not a register access");
        };
        let cmd = u32::from_be_bytes(cmd[..4].try_into().unwrap());
        let addr = (cmd >> 5) & 0x00ff_ffff;
        match data {
            Operation::Write(bytes) => {
                assert_eq!(cmd >> 29, 0b010);
                if regs.fails(addr) {
                    return Err(ErrorKind::Other);
                }
                for (a, b) in (addr..).zip(bytes.iter()) {
                    regs.mem.insert(a, *b);
                }
                regs.written(addr);
            }
            Operation::Read(buf) => {
                assert_eq!(cmd >> 29, 0b011);
                for (a, b) in (addr..).zip(buf.iter_mut()) {
                    *b = *regs.mem.get(&a).unwrap_or(&0);
                }
            }
            _ => panic!("not a register access"),
        }
        Ok(())
    }
}

struct NoDelay;

impl embedded_hal_async::delay::DelayNs for NoDelay {
    async fn delay_ns(&mut self, _ns: u32) {}
}

/// IRQ_N, never asserted.
struct Irq;

impl embedded_hal::digital::ErrorType for Irq {
    type Error = Infallible;
}

impl embedded_hal::digital::InputPin for Irq {
    fn is_high(&mut self) -> Result<bool, Infallible> {
        Ok(true)
    }

    fn is_low(&mut self) -> Result<bool, Infallible> {
        Ok(false)
    }
}

impl embedded_hal_async::digital::Wait for Irq {
    async fn wait_for_high(&mut self) -> Result<(), Infallible> {
        Ok(())
    }

    async fn wait_for_low(&mut self) -> Result<(), Infallible> {
        core::future::pending().await
    }

    async fn wait_for_rising_edge(&mut self) -> Result<(), Infallible> {
        core::future::pending().await
    }

    async fn wait_for_falling_edge(&mut self) -> Result<(), Infallible> {
        core::future::pending().await
    }

    async fn wait_for_any_edge(&mut self) -> Result<(), Infallible> {
        core::future::pending().await
    }
}

/// A LAN9374 laid out like the switch board's, under its runner. Time stands
/// still, so only the first round does the periodic work.
struct Switch {
    state: &'static SwitchState,
    regs: Rc<RefCell<Regs>>,
    runner: Runner<'static, Bus, NoDelay, Irq>,
    events: PortEvents<'static>,
}

impl Switch {
    /// Before the first round, with T1 PHYs on `t1_ports`.
    fn start(t1_ports: &[(Port, Role)]) -> Self {
        let state: &'static SwitchState = Box::leak(Box::new(SwitchState::new(StateConfig {
            ports: 8,
            host: HOST,
            user_ports: &USER_PORTS,
            chassis_mac: [0x02, 0, 0, 0, 0, 0x10],
            initial_state: StpState::Discarding,
            t1_led_mode: t1::LedMode::Activity,
        })));
        let regs = Rc::new(RefCell::new(Regs::default()));
        let runner = Runner::new(Lan937x::new(Bus(regs.clone()), NoDelay), Irq, state, t1_ports);
        Self {
            state,
            regs,
            runner,
            events: state.subscribe().unwrap(),
        }
    }

    /// Started and settled: every user port discarding, the PHYs initialized.
    fn new(t1_ports: &[(Port, Role)]) -> Self {
        let mut switch = Self::start(t1_ports);
        switch.settle();
        assert_eq!(switch.event(), None);
        switch
    }

    fn round(&mut self) -> Result<(), Error<ErrorKind>> {
        embassy_futures::block_on(self.runner.service())
    }

    /// Rounds, which must succeed, until no T1 PHY is initializing.
    fn settle(&mut self) {
        for _ in 0..1000 {
            self.round().unwrap();
            if !self.runner.t1_init_pending() {
                return;
            }
        }
        panic!("the T1 PHYs never finished initializing");
    }

    /// Request forwarding on `port`, and apply it.
    fn open(&mut self, port: Port) {
        self.state.set_stp_state(PortId(port.0), StpState::Forwarding);
        self.round().unwrap();
        assert_eq!(self.event(), Some(applied(port, StpState::Forwarding)));
    }

    fn fail_writes(&self, addr: u32, skip: usize, count: usize) {
        self.regs.borrow_mut().fail_writes(addr, skip, count);
    }

    fn event(&mut self) -> Option<PortEvent> {
        self.events.try_next()
    }

    fn applied(&self, port: Port) -> StpState {
        self.state.port_status(port).stp.applied
    }

    fn stp_register(&self, port: Port) -> u8 {
        self.regs.borrow().get(reg::port(port, reg::P_STP_STATE), 1) as u8
    }

    fn membership(&self, port: Port) -> u8 {
        self.regs.borrow().get(reg::port(port, reg::P_MEMBERSHIP), 4) as u8
    }

    fn phy(&self, port: Port, phy_reg: u8) -> u16 {
        self.regs.borrow().phy[&reg::t1_phy(port, phy_reg)]
    }
}

fn uplink_up() -> ExternalLink {
    ExternalLink {
        up: true,
        speed: Speed::Mbps100,
        duplex: Duplex::Full,
        tx_pause: false,
        rx_pause: false,
    }
}

fn applied(port: Port, state: StpState) -> PortEvent {
    PortEvent::StpApplied {
        port: PortId(port.0),
        state,
    }
}

/// The state register's value for `state`, with learning per the state.
fn register(state: StpState) -> u8 {
    crate::StpState::from(state).register()
}

const HOST_ONLY: u8 = 1 << HOST.0;

/// An opening is reported before it's written. A write that fails leaves it
/// outstanding: the next round makes it, masks and all.
#[test]
fn failed_opening_is_retried() {
    for addr in [
        reg::port(Port(0), reg::P_MEMBERSHIP),
        // After port 0's mask is written.
        reg::port(Port(1), reg::P_MEMBERSHIP),
        reg::port(Port(0), reg::P_STP_STATE),
    ] {
        let mut sw = Switch::new(&[]);
        sw.open(Port(1));
        sw.state.set_stp_state(PortId(0), StpState::Forwarding);
        sw.fail_writes(addr, 0, 1);
        assert!(sw.round().is_err());
        assert_eq!(sw.applied(Port(0)), StpState::Forwarding);
        assert_eq!(sw.event(), Some(applied(Port(0), StpState::Forwarding)));
        assert_eq!(sw.stp_register(Port(0)), register(StpState::Discarding));

        sw.round().unwrap();
        assert_eq!(sw.stp_register(Port(0)), register(StpState::Forwarding));
        assert_eq!(sw.membership(Port(0)), 1 << 1 | HOST_ONLY);
        assert_eq!(sw.membership(Port(1)), 1 << 0 | HOST_ONLY);
        assert_eq!(sw.membership(HOST), !HOST_ONLY);
        // Reported once.
        assert_eq!(sw.event(), None);
    }
}

/// A closing is reported once it's written: until a failed write is made
/// again, the port still forwards as far as spanning tree knows.
#[test]
fn failed_closing_is_retried() {
    for addr in [
        reg::port(Port(0), reg::P_STP_STATE),
        // After the state is written.
        reg::port(Port(0), reg::P_MEMBERSHIP),
    ] {
        let mut sw = Switch::new(&[]);
        sw.open(Port(0));
        sw.open(Port(1));
        sw.state.set_stp_state(PortId(0), StpState::Discarding);
        sw.fail_writes(addr, 0, 1);
        assert!(sw.round().is_err());
        assert_eq!(sw.applied(Port(0)), StpState::Forwarding);
        assert_eq!(sw.event(), None);

        sw.round().unwrap();
        assert_eq!(sw.applied(Port(0)), StpState::Discarding);
        assert_eq!(sw.event(), Some(applied(Port(0), StpState::Discarding)));
        assert_eq!(sw.stp_register(Port(0)), register(StpState::Discarding));
        assert_eq!(sw.membership(Port(0)), HOST_ONLY);
        assert_eq!(sw.membership(Port(1)), HOST_ONLY);
    }
}

/// A newer request supersedes one whose writes failed.
#[test]
fn failed_change_is_superseded() {
    // An opening that failed, then closed again: the port never forwards.
    let mut sw = Switch::new(&[]);
    sw.state.set_stp_state(PortId(0), StpState::Forwarding);
    sw.fail_writes(reg::port(Port(0), reg::P_STP_STATE), 0, 1);
    assert!(sw.round().is_err());
    sw.state.set_stp_state(PortId(0), StpState::Discarding);
    sw.round().unwrap();
    assert_eq!(sw.event(), Some(applied(Port(0), StpState::Forwarding)));
    assert_eq!(sw.event(), Some(applied(Port(0), StpState::Discarding)));
    assert_eq!(sw.stp_register(Port(0)), register(StpState::Discarding));
    assert_eq!(sw.membership(Port(0)), HOST_ONLY);

    // A closing that failed half way, then opened again: the port forwards,
    // as reported all along.
    let mut sw = Switch::new(&[]);
    sw.open(Port(0));
    sw.open(Port(1));
    sw.state.set_stp_state(PortId(0), StpState::Discarding);
    sw.fail_writes(reg::port(Port(0), reg::P_MEMBERSHIP), 0, 1);
    assert!(sw.round().is_err());
    assert_eq!(sw.stp_register(Port(0)), register(StpState::Discarding));
    sw.state.set_stp_state(PortId(0), StpState::Forwarding);
    sw.round().unwrap();
    assert_eq!(sw.applied(Port(0)), StpState::Forwarding);
    assert_eq!(sw.event(), None);
    assert_eq!(sw.stp_register(Port(0)), register(StpState::Forwarding));
    assert_eq!(sw.membership(Port(0)), 1 << 1 | HOST_ONLY);
}

/// A flush that fails is retried until it's done, and only then reported.
/// Learning is back on afterwards, however it failed.
#[test]
fn failed_flush_is_retried() {
    let stp = reg::port(Port(0), reg::P_STP_STATE);
    // It times out.
    retry_flush(|regs| regs.flush_stuck = 1, true);
    // Starting it fails.
    retry_flush(|regs| regs.fail_writes(reg::GLB_SW_LUE_CTL1, 0, 1), true);
    // Turning learning off fails.
    retry_flush(|regs| regs.fail_writes(stp, 0, 1), true);
    // Turning it back on fails: the runner writes the state again.
    retry_flush(|regs| regs.fail_writes(stp, 1, 1), false);
}

/// Flush forwarding port 0 with `fault`, after which learning is on or not
/// (`learning`), then again.
fn retry_flush(fault: impl FnOnce(&mut Regs), learning: bool) {
    let mut sw = Switch::new(&[]);
    sw.open(Port(0));
    sw.state.flush(PortId(0));
    fault(&mut sw.regs.borrow_mut());
    assert!(sw.round().is_err());
    assert!(sw.state.flush_pending(PortId(0)));
    assert_eq!(sw.event(), None);
    assert_eq!(sw.stp_register(Port(0)) & reg::STP_LEARN_DIS == 0, learning);

    sw.round().unwrap();
    assert!(!sw.state.flush_pending(PortId(0)));
    assert_eq!(sw.event(), Some(PortEvent::FlushDone { port: PortId(0) }));
    assert_eq!(sw.stp_register(Port(0)), register(StpState::Forwarding));
}

/// Disabling a T1 port powers its PHY down, and enabling it powers it up and
/// reinitializes it. A power change that fails is retried.
#[test]
fn failed_power_change_is_retried() {
    let mut sw = Switch::new(&[(T1, Role::Master)]);
    let powered_down = |sw: &Switch| sw.phy(T1, t1::BMCR) & t1::BMCR_POWER_DOWN != 0;
    // Every PHY register write goes through the data register.
    sw.state.set_port_enabled(T1, false);
    sw.fail_writes(reg::VPHY_IND_DATA, 0, 1);
    assert!(sw.round().is_err());
    assert!(!powered_down(&sw));
    sw.round().unwrap();
    assert!(powered_down(&sw));

    sw.state.set_port_enabled(T1, true);
    sw.fail_writes(reg::VPHY_IND_DATA, 0, 1);
    assert!(sw.round().is_err());
    assert!(powered_down(&sw));
    assert!(!sw.runner.t1_init_pending());
    sw.round().unwrap();
    assert!(!powered_down(&sw));
    assert!(sw.runner.t1_init[T1.0 as usize].is_some());
    sw.settle();
}

/// Interrupts are enabled once the T1 PHYs are initialized, even if the
/// first attempt fails.
#[test]
fn failed_interrupt_enable_is_retried() {
    let mut sw = Switch::start(&[(T1, Role::Slave)]);
    sw.fail_writes(reg::GLB_PINT_MSK, 0, 1);
    let mut failures = 0;
    for _ in 0..1000 {
        failures += sw.round().is_err() as usize;
        if !sw.runner.t1_init_pending() {
            break;
        }
    }
    assert_eq!(failures, 1);
    assert!(sw.runner.interrupts_enabled);
}

/// An external PHY's link change reaches the switch MAC even if the first
/// attempt fails, and is reported once it has.
#[test]
fn failed_external_link_is_retried() {
    let mut sw = Switch::new(&[]);
    sw.state.set_external_link(UPLINK, uplink_up());
    let ctl0 = reg::port(UPLINK, reg::P_XMII_CTL0);
    sw.fail_writes(ctl0, 0, 1);
    assert!(sw.round().is_err());
    assert!(!sw.state.port_status(UPLINK).link.up);
    assert_eq!(sw.event(), None);

    sw.round().unwrap();
    assert!(sw.state.port_status(UPLINK).link.up);
    assert!(matches!(sw.event(), Some(PortEvent::LinkUp { port, .. }) if port == PortId(UPLINK.0)));
    assert_eq!(
        sw.regs.borrow().get(ctl0, 1) as u8,
        reg::XMII_CTL0_FULL_DUPLEX | reg::XMII_CTL0_100
    );
}

/// A loopback change that fails is retried, learning included.
#[test]
fn failed_loopback_is_retried() {
    let mut sw = Switch::new(&[(T1, Role::Master)]);
    sw.open(T1);
    let looped = |sw: &Switch| sw.phy(T1, t1::BMCR) & t1::BMCR_LOOPBACK != 0;
    sw.state.set_t1_loopback(T1, true);
    sw.fail_writes(reg::VPHY_IND_DATA, 0, 1);
    assert!(sw.round().is_err());
    assert!(!looped(&sw));
    sw.round().unwrap();
    assert!(looped(&sw));
    // A looped port doesn't learn.
    assert_eq!(sw.stp_register(T1), register(StpState::Forwarding) | reg::STP_LEARN_DIS);

    sw.state.set_t1_loopback(T1, false);
    sw.fail_writes(reg::port(T1, reg::P_STP_STATE), 0, 1);
    assert!(sw.round().is_err());
    assert!(!looped(&sw));
    sw.round().unwrap();
    assert_eq!(sw.stp_register(T1), register(StpState::Forwarding));
}

/// Work that keeps failing doesn't hold up other work.
#[test]
fn failing_work_does_not_hold_up_the_rest() {
    let mut sw = Switch::new(&[]);
    sw.open(Port(0));
    // Port 0's state register can't be written any more.
    sw.fail_writes(reg::port(Port(0), reg::P_STP_STATE), 0, usize::MAX);
    sw.state.flush(PortId(0));
    sw.state.flush(PortId(1));
    assert!(sw.round().is_err());
    assert!(sw.state.flush_pending(PortId(0)));
    assert!(!sw.state.flush_pending(PortId(1)));

    // Now writing port 0's state again fails first.
    sw.state.set_external_link(UPLINK, uplink_up());
    assert!(sw.round().is_err());
    assert!(sw.state.flush_pending(PortId(0)));
    assert!(sw.state.port_status(UPLINK).link.up);
}
