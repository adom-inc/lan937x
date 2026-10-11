//! Spanning-tree port states and port membership.

use crate::{Port, reg};

/// A port's spanning-tree state, as 802.1D-2004 defines them.
///
/// The variants are ordered from most to least restrictive.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum StpState {
    /// Out of the spanning tree: nothing received, sent or learned.
    Disabled,
    /// Nothing forwarded, nothing learned.
    Discarding,
    /// Received frames are learned from but not forwarded.
    Learning,
    /// Frames are received, forwarded and learned from.
    Forwarding,
}

impl StpState {
    /// The `P_STP_STATE` register value (Linux `ksz_port_stp_state_set`).
    pub const fn register(self) -> u8 {
        match self {
            StpState::Disabled | StpState::Discarding => reg::STP_LEARN_DIS,
            StpState::Learning => reg::STP_RX_EN,
            StpState::Forwarding => reg::STP_RX_EN | reg::STP_TX_EN,
        }
    }
}

/// The membership masks (`P_MEMBERSHIP`) for a set of port states, with all
/// user ports in one bridge.
///
/// - The host port may send to every user port.
/// - A forwarding user port may send to the other forwarding user ports and the host.
/// - Any other user port may send to the host only. That keeps a learning
///   port's received frames from being forwarded, while the host still sees
///   its port-local frames.
///
/// `ports` is the number of ports; `forwarding` has bit n set for each
/// forwarding user port n. Returns one mask per port.
pub fn membership(ports: u8, host: Port, forwarding: u8) -> [u8; 8] {
    let all = if ports >= 8 { 0xff } else { (1u8 << ports) - 1 };
    let host_bit = 1u8 << host.0;
    let mut masks = [0; 8];
    for port in 0..ports.min(8) {
        let bit = 1u8 << port;
        masks[port as usize] = if bit == host_bit {
            all & !host_bit
        } else if forwarding & bit != 0 {
            (forwarding & !bit & !host_bit) | host_bit
        } else {
            host_bit
        };
    }
    masks
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn registers() {
        assert_eq!(StpState::Disabled.register(), 0x01);
        assert_eq!(StpState::Discarding.register(), 0x01);
        assert_eq!(StpState::Learning.register(), 0x02);
        assert_eq!(StpState::Forwarding.register(), 0x06);
    }

    #[test]
    fn memberships() {
        let host = Port(5);
        // Ports 0 and 4 forwarding, port 1 learning or blocked.
        let masks = membership(8, host, 0b0001_0001);
        assert_eq!(masks[5], 0b1101_1111);
        assert_eq!(masks[0], 0b0011_0000);
        assert_eq!(masks[4], 0b0010_0001);
        assert_eq!(masks[1], 0b0010_0000);
        // Nothing forwarding.
        assert_eq!(membership(8, host, 0)[4], 0b0010_0000);
    }
}
