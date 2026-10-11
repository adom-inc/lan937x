#![cfg_attr(not(test), no_std)]
//! Driver for the Microchip LAN937x automotive Ethernet switches (LAN9370,
//! LAN9371, LAN9372, LAN9373, LAN9374): 100BASE-T1 ports with integrated
//! PHYs, xMII ports, and PTP hardware.
//!
//! - [`Lan937x`]: the switch over SPI, with its integrated T1 PHYs reached
//!   through the indirect SPI window ([`t1`]).
//! - [`tag`]: the host port's tail tag.
//! - With the `xarxa` feature: shared switch state and the task that services
//!   the switch, implementing `xarxa-dsa`'s traits so the switch can sit under
//!   an embassy-net interface.

pub(crate) mod fmt;

mod device;
pub mod reg;
pub mod spi;
pub mod stp;
pub mod t1;
pub mod tag;

pub mod ptp;

#[cfg(feature = "xarxa")]
pub mod xarxa;

pub use device::{ChipId, Config, Lan937x, Xmii, delay_codes, lan9374_afe, mib};
pub use stp::StpState;

/// A switch port, numbered from zero: `Port(0)` is port 1 in the datasheet.
///
/// This is the numbering of the tail tag, Linux and `xarxa-dsa`.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Port(pub u8);

impl Port {
    /// The port number the datasheet uses (1-based).
    pub const fn datasheet_number(self) -> u8 {
        self.0 + 1
    }
}

/// Link speed.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Speed {
    /// 10 Mbit/s.
    Mbps10,
    /// 100 Mbit/s.
    Mbps100,
    /// 1000 Mbit/s.
    Mbps1000,
}

/// Link duplex.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Duplex {
    /// Half duplex.
    Half,
    /// Full duplex.
    Full,
}

/// An error.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<E> {
    /// The SPI bus failed.
    Spi(E),
    /// The chip ID isn't a LAN937x's.
    UnknownChip(u32),
    /// The switch didn't finish an operation in time.
    Timeout,
}

impl<E> Error<E> {
    /// What went wrong, without the bus error itself.
    pub fn kind(&self) -> &'static str {
        match self {
            Error::Spi(_) => "spi",
            Error::UnknownChip(_) => "unknown chip",
            Error::Timeout => "timeout",
        }
    }
}
