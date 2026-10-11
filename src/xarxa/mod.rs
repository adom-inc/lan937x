//! The switch under an embassy-net interface, with `xarxa-dsa`.
//!
//! - [`TAG`]: the host port's tail tag as a [`TagProtocol`].
//! - [`SwitchState`]: the switch's shared state, implementing [`DsaSwitch`]
//!   (for `xarxa_dsa::DsaDriver`) and [`SwitchPorts`] (for protocol agents).
//!   Every method is synchronous and cheap; requests are carried out by:
//! - [`Runner`]: the task that owns the SPI bus and the switch's interrupt
//!   line. It applies spanning-tree states and flushes, services the T1 PHYs
//!   (link interrupts, initialization) and PTP transmit timestamps, keeps the
//!   clock reading used to rebuild receive timestamps fresh, programs xMII
//!   ports for external PHY link changes, and accumulates the counters.
//!
//! [`DsaSwitch`]: xarxa_dsa::DsaSwitch
//! [`SwitchPorts`]: xarxa_dsa::SwitchPorts

mod runner;
mod state;

pub use runner::Runner;
pub use state::{ExternalLink, PortCounters, PortStatus, StateConfig, SwitchState};
use xarxa_driver::PacketBuf;
use xarxa_dsa::{Egress, PortId, RxTag, TagOverhead, TagProtocol, TxTag};

use crate::tag;

/// The LAN937x tail tag, in PTP mode (which [`Lan937x::init`](crate::Lan937x::init) sets).
pub struct Lan937xTag;

/// The LAN937x tail tag.
pub static TAG: Lan937xTag = Lan937xTag;

impl TagProtocol for Lan937xTag {
    fn overhead(&self) -> TagOverhead {
        TagOverhead {
            tx_head: 0,
            tx_tail: tag::TX_TAIL_LEN as u8,
            rx_head_max: 0,
            rx_tail_max: tag::RX_TAIL_MAX as u8,
        }
    }

    fn decode(&self, buf: &mut PacketBuf) -> Option<RxTag> {
        let tail = tag::decode(buf)?;
        buf.set_len(tail.len);
        Some(RxTag::new(PortId(tail.port.0), tail.timestamp))
    }

    fn encode(&self, buf: &mut PacketBuf, tx: &TxTag) {
        let destination = match tx.egress {
            Egress::Lookup => tag::Destination::Lookup,
            Egress::Port(port) => tag::Destination::Ports {
                mask: 1 << port.0,
                override_port_state: tx.bypass_port_state,
            },
        };
        let len = buf.len();
        let tagged = len.max(tag::MIN_FRAME_LEN) + tag::TX_TAIL_LEN;
        buf.set_len(tagged);
        tag::encode(buf, len, tag::tx_tag(destination, tx.priority));
    }

    fn supports_lookup(&self) -> bool {
        true
    }
}

impl From<crate::StpState> for xarxa_dsa::StpState {
    fn from(state: crate::StpState) -> Self {
        match state {
            crate::StpState::Disabled => Self::Disabled,
            crate::StpState::Discarding => Self::Discarding,
            crate::StpState::Learning => Self::Learning,
            crate::StpState::Forwarding => Self::Forwarding,
        }
    }
}

impl From<xarxa_dsa::StpState> for crate::StpState {
    fn from(state: xarxa_dsa::StpState) -> Self {
        match state {
            xarxa_dsa::StpState::Disabled => Self::Disabled,
            xarxa_dsa::StpState::Discarding => Self::Discarding,
            xarxa_dsa::StpState::Learning => Self::Learning,
            xarxa_dsa::StpState::Forwarding => Self::Forwarding,
        }
    }
}

impl From<crate::Speed> for xarxa_dsa::Speed {
    fn from(speed: crate::Speed) -> Self {
        match speed {
            crate::Speed::Mbps10 => Self::Mbps10,
            crate::Speed::Mbps100 => Self::Mbps100,
            crate::Speed::Mbps1000 => Self::Mbps1000,
        }
    }
}

impl From<crate::Duplex> for xarxa_dsa::Duplex {
    fn from(duplex: crate::Duplex) -> Self {
        match duplex {
            crate::Duplex::Half => Self::Half,
            crate::Duplex::Full => Self::Full,
        }
    }
}
