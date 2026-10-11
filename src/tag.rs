//! The tail tag on the host port.
//!
//! The host port is the one port with tail tagging enabled. Frames to and from
//! the host carry a tag after the payload (and after any padding), before the
//! FCS:
//!
//! ```text
//! host → switch:  DA | SA | … | payload | ts (4) | tag (2) | FCS
//! switch → host:  DA | SA | … | payload | [ts (4)] | tag0 (1) | FCS
//! ```
//!
//! The 4-byte field on frames from the host is present whenever the switch is
//! in PTP mode (`PTP_MSG_CONF1.PTP_ENABLE`), on every frame. It carries the
//! ingress timestamp for one-step peer delay responses and is zero otherwise.
//! This driver always runs the switch in PTP mode.
//!
//! On frames to the host, a timestamp is present if bit 7 of `tag0` is set: the
//! partial receive time, bits 31:30 seconds modulo 4 and bits 29:0 nanoseconds.

use crate::Port;

/// Bytes added to frames from the host: the timestamp field and the tag.
pub const TX_TAIL_LEN: usize = 6;
/// Most bytes added to frames to the host.
pub const RX_TAIL_MAX: usize = 5;
/// Minimum Ethernet frame length without the FCS. Frames are padded to it
/// before the tail goes on, so the switch finds the tail at the end.
pub const MIN_FRAME_LEN: usize = 60;

/// The tag is valid. Always set.
pub const TX_VALID: u16 = 1 << 13;
/// Forward by the address table and port states; the port mask is ignored.
pub const TX_LOOKUP: u16 = 1 << 12;
/// Send even if a destination port's spanning-tree state blocks it.
pub const TX_OVERRIDE: u16 = 1 << 11;
const TX_PRIO_SHIFT: u16 = 8;

/// A timestamp precedes the tag.
pub const RX_PTP: u8 = 1 << 7;
/// The frame comes from another chip of a cascade.
pub const RX_CASCADE: u8 = 1 << 6;
const RX_PORT: u8 = 0b111;

/// Where a frame from the host goes.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Destination {
    /// By the address table and port states.
    Lookup,
    /// To these ports (bit n = [`Port`] n).
    Ports {
        /// Port mask.
        mask: u8,
        /// Send even to ports whose spanning-tree state blocks it.
        override_port_state: bool,
    },
}

/// The 16-bit tag for a frame from the host.
pub const fn tx_tag(destination: Destination, priority: Option<u8>) -> u16 {
    let prio = match priority {
        Some(p) => ((p & 0x7) as u16) << TX_PRIO_SHIFT,
        None => 0,
    };
    match destination {
        Destination::Lookup => TX_VALID | TX_LOOKUP | prio,
        Destination::Ports {
            mask,
            override_port_state,
        } => TX_VALID | if override_port_state { TX_OVERRIDE } else { 0 } | prio | mask as u16,
    }
}

/// Pad `frame[..len]` to the minimum frame length and append the tail for
/// `tag`, in place. Returns the new length.
///
/// # Panics
/// If `frame` is shorter than the result.
pub fn encode(frame: &mut [u8], len: usize, tag: u16) -> usize {
    let padded = len.max(MIN_FRAME_LEN);
    frame[len..padded].fill(0);
    frame[padded..padded + 4].fill(0);
    frame[padded + 4..padded + 6].copy_from_slice(&tag.to_be_bytes());
    padded + TX_TAIL_LEN
}

/// What the tail of a frame to the host says.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RxTail {
    /// The port the frame arrived on.
    pub port: Port,
    /// The partial receive timestamp, if any.
    pub timestamp: Option<u32>,
    /// The frame's length without the tail.
    pub len: usize,
}

/// Parse the tail of a frame to the host (FCS already removed).
///
/// `None` for frames too short to hold their tail, and for frames from
/// another chip of a cascade, which this driver doesn't support.
pub fn decode(frame: &[u8]) -> Option<RxTail> {
    let (&tag, rest) = frame.split_last()?;
    if tag & RX_CASCADE != 0 {
        return None;
    }
    let port = Port(tag & RX_PORT);
    if tag & RX_PTP != 0 {
        let len = rest.len().checked_sub(4)?;
        let timestamp = u32::from_be_bytes(rest[len..].try_into().unwrap());
        Some(RxTail {
            port,
            timestamp: Some(timestamp),
            len,
        })
    } else {
        Some(RxTail {
            port,
            timestamp: None,
            len: rest.len(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tx_tags() {
        assert_eq!(tx_tag(Destination::Lookup, None), 0x3000);
        assert_eq!(
            tx_tag(
                Destination::Ports {
                    mask: 1 << 4,
                    override_port_state: false
                },
                None
            ),
            0x2010
        );
        // Linux lan937x_xmit: link-local frames get the override.
        assert_eq!(
            tx_tag(
                Destination::Ports {
                    mask: 1 << 0,
                    override_port_state: true
                },
                Some(7)
            ),
            0x2f01
        );
    }

    #[test]
    fn encode_pads_short_frames_before_the_tail() {
        let mut frame = [0xaa; 80];
        let len = encode(&mut frame, 42, 0x3000);
        assert_eq!(len, 66);
        assert!(frame[42..60].iter().all(|&b| b == 0));
        assert_eq!(&frame[60..66], &[0, 0, 0, 0, 0x30, 0x00]);
    }

    #[test]
    fn encode_full_size() {
        let mut frame = [0x55; 1520];
        assert_eq!(encode(&mut frame, 1514, 0x2001), 1520);
        assert_eq!(frame[1513], 0x55);
        assert_eq!(&frame[1514..], &[0, 0, 0, 0, 0x20, 0x01]);
    }

    #[test]
    fn decode_tails() {
        let mut frame = [0u8; 61];
        frame[60] = 4;
        assert_eq!(
            decode(&frame),
            Some(RxTail {
                port: Port(4),
                timestamp: None,
                len: 60
            })
        );

        let mut frame = [0u8; 65];
        frame[60..64].copy_from_slice(&0xc000_0123u32.to_be_bytes());
        frame[64] = RX_PTP | 7;
        assert_eq!(
            decode(&frame),
            Some(RxTail {
                port: Port(7),
                timestamp: Some(0xc000_0123),
                len: 60
            })
        );
    }

    #[test]
    fn decode_rejects_malformed() {
        assert_eq!(decode(&[]), None);
        assert_eq!(decode(&[0, 0, 0, RX_PTP]), None);
        assert_eq!(
            decode(&[0; 60].iter().chain(&[RX_CASCADE]).copied().collect::<Vec<_>>()),
            None
        );
    }
}
