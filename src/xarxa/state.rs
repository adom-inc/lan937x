use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, Ordering};
use core::task::Waker;

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex as AsyncMutex;
use embassy_sync::signal::Signal;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{Duration, Instant};
use heapless::Deque;
use xarxa_driver::{Timestamp, TxTimestamp};
use xarxa_dsa::{
    DsaSwitch, Duplex, PortEvent, PortEventChannel, PortEvents, PortId, PortInfo, PortLink, Speed, StpState, StpView,
    SwitchPorts,
};

use crate::Port;
use crate::ptp::{self, PhcTime};
use crate::t1::{Bank, LedMode, Role};

type M = CriticalSectionRawMutex;

/// Ports a LAN937x has at most.
pub(crate) const MAX_PORTS: usize = 8;
/// A transmit timestamp request is dropped if its interrupt doesn't come within this.
pub(crate) const TX_TIMESTAMP_TIMEOUT: Duration = Duration::from_millis(100);
/// Receive timestamps aren't rebuilt from a clock reading older than this.
const PHC_REFERENCE_MAX_AGE: Duration = Duration::from_secs(10);

/// Configuration of [`SwitchState`].
#[derive(Debug, Clone)]
pub struct StateConfig {
    /// The number of ports.
    pub ports: u8,
    /// The host port.
    pub host: Port,
    /// The user ports: every port but the host, with their names and kinds.
    pub user_ports: &'static [PortInfo],
    /// The chassis MAC address. Port `n`'s MAC is this plus `n + 1` (in the
    /// last byte, which must have room).
    pub chassis_mac: [u8; 6],
    /// The spanning-tree state the user ports start in, matching
    /// [`Config::initial_state`](crate::Config::initial_state).
    pub initial_state: StpState,
    /// What the T1 PHYs' LEDs show.
    pub t1_led_mode: LedMode,
}

/// A register width, for [`DiagRequest`].
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegWidth {
    U8,
    U16,
    U32,
}

/// A raw access for diagnostics, carried out by the runner between its own
/// work. See [`SwitchState::diag`].
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagRequest {
    /// Read a switch register.
    ReadReg { addr: u32, width: RegWidth },
    /// Write a switch register.
    WriteReg { addr: u32, width: RegWidth, value: u32 },
    /// Read a register of a T1 port's PHY.
    ReadPhy { port: Port, bank: Bank, reg: u8 },
    /// Write a register of a T1 port's PHY.
    WritePhy {
        port: Port,
        bank: Bank,
        reg: u8,
        value: u16,
    },
    /// Read a raw MIB counter (which clears it; the runner's totals miss it).
    ReadMib { port: Port, index: u8 },
    /// A T1 port's signal quality, 0 (worst) to 7.
    Sqi { port: Port },
}

/// Why a [`DiagRequest`] failed.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DiagError {
    /// The switch didn't finish in time.
    Timeout,
    /// The bus failed.
    Bus,
    /// The port has no integrated T1 PHY.
    NotT1Port,
}

/// A link change of an external PHY on an xMII port, from the task that
/// services that PHY.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExternalLink {
    /// Link up.
    pub up: bool,
    /// Speed.
    pub speed: crate::Speed,
    /// Duplex.
    pub duplex: crate::Duplex,
    /// The port may send pause frames.
    pub tx_pause: bool,
    /// The port honours received pause frames.
    pub rx_pause: bool,
}

/// Counters of one port, accumulated from the switch's clear-on-read MIB.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[allow(missing_docs)]
pub struct PortCounters {
    pub rx_bytes: u64,
    pub tx_bytes: u64,
    pub rx_unicast: u64,
    pub rx_multicast: u64,
    pub rx_broadcast: u64,
    pub tx_unicast: u64,
    pub tx_multicast: u64,
    pub tx_broadcast: u64,
    pub rx_drops: u64,
    pub tx_drops: u64,
    pub rx_crc_errors: u64,
}

/// A snapshot of a port, for management.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PortStatus {
    /// Administratively enabled.
    pub enabled: bool,
    /// The physical link.
    pub link: PortLink,
    /// Spanning-tree state.
    pub stp: StpView,
    /// The T1 PHY's configured role, for T1 ports.
    pub role: Option<Role>,
    /// Counters.
    pub counters: PortCounters,
}

#[derive(Clone, Copy)]
pub(crate) struct PortState {
    pub(crate) enabled: bool,
    pub(crate) stp: StpView,
    pub(crate) link: PortLink,
    pub(crate) flush_requested: bool,
    pub(crate) flush_pending: bool,
    /// For T1 ports: the configured role.
    pub(crate) role: Option<Role>,
    /// The T1 PHY must be (re)initialized.
    pub(crate) reinit: bool,
    /// A requested change of the T1 PHY's loopback.
    pub(crate) loopback: Option<bool>,
    /// A requested power change of the T1 PHY (true: powered), from enabling
    /// or disabling the port.
    pub(crate) power: Option<bool>,
    pub(crate) counters: PortCounters,
}

#[derive(Clone, Copy)]
pub(crate) struct PendingTimestamp {
    pub(crate) id: u32,
    pub(crate) since: Instant,
}

pub(crate) struct Inner {
    pub(crate) ports: [PortState; MAX_PORTS],
    pub(crate) external_links: [Option<ExternalLink>; MAX_PORTS],
    pub(crate) pending_timestamps: [[Option<PendingTimestamp>; 3]; MAX_PORTS],
    pub(crate) tx_timestamps: Deque<TxTimestamp, 8>,
    /// A reading of the switch clock and when it was taken.
    pub(crate) phc_reference: Option<(PhcTime, Instant)>,
    pub(crate) phc_step: Option<i64>,
    pub(crate) phc_rate: Option<i64>,
    pub(crate) led_mode: LedMode,
    /// Ports whose PHY LED mode must be (re)written.
    pub(crate) led_pending: u8,
}

/// The shared state of a LAN937x. See the [module documentation](super).
///
/// Lives in a `static` (`StaticCell`), shared by the network stack (through
/// `DsaDriver`), the protocol agents and the [`Runner`](super::Runner).
pub struct SwitchState {
    pub(crate) config: StateConfig,
    pub(crate) inner: Mutex<M, RefCell<Inner>>,
    events: PortEventChannel<M, 16, 6>,
    stack_waker: AtomicWaker,
    /// Wakes the runner: a request needs carrying out.
    pub(crate) work: Signal<M, ()>,
    diag_lock: AsyncMutex<M, ()>,
    pub(crate) diag_request: Signal<M, DiagRequest>,
    pub(crate) diag_response: Signal<M, Result<u64, DiagError>>,
    rounds: AtomicU32,
}

impl SwitchState {
    /// The state of a switch configured with `config`.
    pub fn new(config: StateConfig) -> Self {
        let port = PortState {
            enabled: true,
            stp: StpView::settled(config.initial_state),
            link: PortLink::DOWN,
            flush_requested: false,
            flush_pending: false,
            role: None,
            reinit: false,
            loopback: None,
            power: None,
            counters: PortCounters::default(),
        };
        let mut ports = [port; MAX_PORTS];
        ports[config.host.0 as usize].stp = StpView::settled(StpState::Forwarding);
        Self {
            inner: Mutex::new(RefCell::new(Inner {
                ports,
                external_links: [None; MAX_PORTS],
                pending_timestamps: [[None; 3]; MAX_PORTS],
                tx_timestamps: Deque::new(),
                phc_reference: None,
                phc_step: None,
                phc_rate: None,
                led_mode: config.t1_led_mode,
                led_pending: 0,
            })),
            config,
            events: PortEventChannel::new(),
            stack_waker: AtomicWaker::new(),
            work: Signal::new(),
            diag_lock: AsyncMutex::new(()),
            diag_request: Signal::new(),
            diag_response: Signal::new(),
            rounds: AtomicU32::new(0),
        }
    }

    /// Rounds the runner has done, wrapping: at least two a second while it
    /// runs, for a watchdog to check.
    pub fn rounds(&self) -> u32 {
        self.rounds.load(Ordering::Relaxed)
    }

    pub(crate) fn count_round(&self) {
        self.rounds.fetch_add(1, Ordering::Relaxed);
    }

    pub(crate) fn lock<R>(&self, f: impl FnOnce(&mut Inner) -> R) -> R {
        self.inner.lock(|inner| f(&mut inner.borrow_mut()))
    }

    pub(crate) fn publish(&self, event: PortEvent) {
        self.events.publish(event);
        self.stack_waker.wake();
    }

    pub(crate) fn wake_stack(&self) {
        self.stack_waker.wake();
    }

    fn index(&self, port: PortId) -> Option<usize> {
        (port.0 < self.config.ports).then_some(port.0 as usize)
    }

    /// Report a link change of the external PHY on xMII port `port`. The
    /// runner programs the port's MAC to match, then publishes the change.
    pub fn set_external_link(&self, port: Port, link: ExternalLink) {
        self.lock(|inner| inner.external_links[port.0 as usize] = Some(link));
        self.work.signal(());
    }

    /// Administratively enable or disable a port. A disabled port's spanning
    /// tree state is `Disabled` in hardware (applied), whatever was requested,
    /// so it neither forwards nor reaches the host, and a T1 port's PHY is
    /// powered down, so the link drops and the other end notices at once.
    /// Enabling powers it up and reinitializes it with its role.
    /// [`PortEvent::Enabled`] reports the change.
    pub fn set_port_enabled(&self, port: Port, enabled: bool) {
        let changed = self.lock(|inner| {
            let state = &mut inner.ports[port.0 as usize];
            let changed = state.enabled != enabled;
            state.enabled = enabled;
            if changed {
                state.power = Some(enabled);
            }
            changed
        });
        if !changed {
            return;
        }
        self.wake_stack();
        self.publish(PortEvent::Enabled {
            port: PortId(port.0),
            enabled,
        });
        // The runner sets the hardware state to match.
        self.work.signal(());
    }

    /// Set a T1 port's master/slave role. Re-initializes its PHY, so the link
    /// drops and retrains.
    pub fn set_t1_role(&self, port: Port, role: Role) {
        self.lock(|inner| {
            let state = &mut inner.ports[port.0 as usize];
            state.role = Some(role);
            state.reinit = true;
        });
        self.work.signal(());
    }

    /// Put a T1 port's PHY in near-end loopback, or take it out, for testing:
    /// frames sent out of the port come back in on it.
    pub fn set_t1_loopback(&self, port: Port, on: bool) {
        self.lock(|inner| inner.ports[port.0 as usize].loopback = Some(on));
        self.work.signal(());
    }

    /// Set what the T1 PHYs' LEDs show.
    pub fn set_t1_led_mode(&self, mode: LedMode) {
        self.lock(|inner| {
            inner.led_mode = mode;
            inner.led_pending = 0xff;
        });
        self.work.signal(());
    }

    /// Carry out a raw access for diagnostics. Reads return the value;
    /// writes return 0. One at a time; others wait.
    pub async fn diag(&self, request: DiagRequest) -> Result<u64, DiagError> {
        let _guard = self.diag_lock.lock().await;
        self.diag_response.reset();
        self.diag_request.signal(request);
        self.work.signal(());
        self.diag_response.wait().await
    }

    /// A snapshot of `port`.
    pub fn port_status(&self, port: Port) -> PortStatus {
        self.lock(|inner| {
            let state = &inner.ports[port.0 as usize];
            PortStatus {
                enabled: state.enabled,
                link: state.link,
                stp: state.stp,
                role: state.role,
                counters: state.counters,
            }
        })
    }

    /// The switch clock's current time, extrapolated from the runner's last
    /// reading. `None` before the first reading.
    pub fn phc_now(&self) -> Option<PhcTime> {
        let (reference, at) = self.lock(|inner| inner.phc_reference)?;
        let elapsed = at.elapsed().as_micros() as i64 * 1000;
        let ns = reference.as_nanos() + elapsed;
        Some(PhcTime {
            sec: (ns / 1_000_000_000) as u32,
            ns: (ns % 1_000_000_000) as u32,
        })
    }

    /// Ask for the switch clock to be stepped by `delta` nanoseconds.
    pub fn request_phc_step(&self, delta: i64) {
        self.lock(|inner| {
            inner.phc_step = Some(inner.phc_step.unwrap_or(0) + delta);
            // Timestamps rebuilt from the old reading would be off.
            inner.phc_reference = None;
        });
        self.work.signal(());
    }

    /// Ask for the switch clock's frequency to be adjusted by `scaled_ppm`
    /// (ppm × 2^16).
    pub fn request_phc_rate(&self, scaled_ppm: i64) {
        self.lock(|inner| inner.phc_rate = Some(scaled_ppm));
        self.work.signal(());
    }
}

impl DsaSwitch for SwitchState {
    fn host_port(&self) -> PortId {
        PortId(self.config.host.0)
    }

    fn port_count(&self) -> u8 {
        self.config.ports
    }

    fn port_enabled(&self, port: PortId) -> bool {
        self.index(port)
            .is_some_and(|i| self.lock(|inner| inner.ports[i].enabled))
    }

    fn stp_view(&self, port: PortId) -> StpView {
        match self.index(port) {
            Some(i) => self.lock(|inner| inner.ports[i].stp),
            None => StpView::settled(StpState::Disabled),
        }
    }

    fn link(&self, port: PortId) -> PortLink {
        match self.index(port) {
            Some(i) => self.lock(|inner| inner.ports[i].link),
            None => PortLink::DOWN,
        }
    }

    fn rx_timestamp(&self, _port: PortId, raw: u32) -> Option<Timestamp> {
        let (reference, at) = self.lock(|inner| inner.phc_reference)?;
        let age = at.elapsed();
        if age > PHC_REFERENCE_MAX_AGE {
            return None;
        }
        // Extrapolate the reading to now, then rebuild around it.
        let ns = reference.as_nanos() + age.as_micros() as i64 * 1000;
        let now = PhcTime {
            sec: (ns / 1_000_000_000) as u32,
            ns: (ns % 1_000_000_000) as u32,
        };
        let time = ptp::reconstruct(raw, now);
        Some(Timestamp::from_seconds_and_nanos(time.sec, time.ns))
    }

    fn request_tx_timestamp(&self, port: PortId, id: u32, frame: &[u8]) -> bool {
        let (Some(i), Some(class)) = (self.index(port), ptp::classify(frame)) else {
            return false;
        };
        // One register per class per port: a request that overwrites an
        // unread one loses it, and its requester times out.
        self.lock(|inner| {
            inner.pending_timestamps[i][class.index()] = Some(PendingTimestamp {
                id,
                since: Instant::now(),
            });
        });
        true
    }

    fn cancel_tx_timestamp(&self, port: PortId, id: u32) {
        let Some(i) = self.index(port) else { return };
        self.lock(|inner| {
            for slot in &mut inner.pending_timestamps[i] {
                if slot.is_some_and(|p| p.id == id) {
                    *slot = None;
                }
            }
        });
    }

    fn poll_tx_timestamp(&self) -> Option<TxTimestamp> {
        self.lock(|inner| inner.tx_timestamps.pop_front())
    }

    fn register_waker(&self, waker: &Waker) {
        self.stack_waker.register(waker);
    }
}

impl SwitchPorts for SwitchState {
    fn ports(&self) -> &[PortInfo] {
        self.config.user_ports
    }

    fn port_mac(&self, port: PortId) -> [u8; 6] {
        let mut mac = self.config.chassis_mac;
        mac[5] = mac[5].wrapping_add(port.0 + 1);
        mac
    }

    fn link(&self, port: PortId) -> PortLink {
        DsaSwitch::link(self, port)
    }

    fn set_stp_state(&self, port: PortId, state: StpState) {
        let Some(i) = self.index(port) else { return };
        if port == self.host_port() {
            return;
        }
        // Takes effect in the wrapper's rules at once; the runner applies it.
        self.lock(|inner| inner.ports[i].stp.requested = state);
        self.work.signal(());
    }

    fn stp_view(&self, port: PortId) -> StpView {
        DsaSwitch::stp_view(self, port)
    }

    fn flush(&self, port: PortId) {
        let Some(i) = self.index(port) else { return };
        self.lock(|inner| {
            inner.ports[i].flush_requested = true;
            inner.ports[i].flush_pending = true;
        });
        self.work.signal(());
    }

    fn enabled(&self, port: PortId) -> bool {
        self.index(port)
            .is_some_and(|i| self.lock(|inner| inner.ports[i].enabled))
    }

    fn flush_pending(&self, port: PortId) -> bool {
        self.index(port)
            .is_some_and(|i| self.lock(|inner| inner.ports[i].flush_pending))
    }

    fn subscribe(&self) -> Option<PortEvents<'_>> {
        self.events.subscribe()
    }
}

pub(crate) fn link_from(link: &ExternalLink) -> PortLink {
    PortLink {
        up: link.up,
        speed: link.speed.into(),
        duplex: link.duplex.into(),
    }
}

pub(crate) const T1_LINK: PortLink = PortLink {
    up: true,
    speed: Speed::Mbps100,
    duplex: Duplex::Full,
};
