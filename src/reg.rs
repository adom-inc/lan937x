//! Register definitions, from Microchip AN4005 (LAN937x register definitions).
//!
//! Addresses are 16 bits: `[15:12]` port (0 = global, 1..8 = port N as the
//! datasheet numbers them), `[11:8]` function page, `[7:0]` register.
//! Registers are big-endian. Port registers are given as offsets; [`port`]
//! turns one into an address.

use crate::Port;

/// The address of port register `offset` of `port`.
pub const fn port(port: Port, offset: u16) -> u32 {
    (((port.0 as u32) + 1) << 12) | offset as u32
}

// Global operation (0x00xx).

/// Chip ID, bytes 0..3: `00`, `93`, the chip (`0x70`–`0x74` for LAN9370–LAN9374),
/// and the revision in bits 7:4 of the last byte.
pub const CHIP_ID: u32 = 0x0000;

/// Global control 0 (8-bit).
pub const GLB_CONTROL0: u32 = 0x0007;
/// Blocks register access to the T1 PHYs, TX PHY and virtual PHY. Default 1.
pub const GLB_CONTROL0_APB_PHY_REG_BLK: u8 = 1 << 7;
/// Switch RAM initialization done.
pub const GLB_CONTROL0_SW_INI_DONE: u8 = 1 << 4;

/// Global interrupt status (32-bit).
pub const GLB_INT_STS: u32 = 0x0010;
/// Global interrupt mask (32-bit), 1 = masked.
pub const GLB_INT_MSK: u32 = 0x0014;
/// Power-on ready (W1C). Holds IRQ_N asserted from power-on until cleared;
/// it can't be masked.
pub const GLB_INT_POR_READY: u32 = 1 << 25;
/// PTP trigger/timestamp units.
pub const GLB_INT_PTP: u32 = 1 << 30;
/// Global port interrupt status (32-bit, read-only): bit n = port n.
pub const GLB_PINT_STS: u32 = 0x0018;
/// Global port interrupt mask (32-bit): bit n = port n, 1 = masked.
pub const GLB_PINT_MSK: u32 = 0x001C;

/// Cascade mode control (32-bit). Bit 15 enables cascade mode (3-byte tail tags).
pub const GLB_CASCADE_MODE_CTL: u32 = 0x0030;
/// Cascade mode enable.
pub const GLB_CASCADE_MODE_EN: u32 = 1 << 15;

// Global I/O (0x01xx).

/// Clock output control (8-bit): bit 1 disables CLK125, bit 0 disables CLKO_25M.
pub const GLB_OUT_CLK_CTL: u32 = 0x0103;

// Global PHY control and straps (0x02xx).

/// Latched strap values (32-bit, read-only).
pub const CFG_STRAP_VAL: u32 = 0x0210;
/// Strap overrides (32-bit).
pub const CFG_STRAP_OVR: u32 = 0x0214;
/// Disables the virtual PHY's effect on its port (port 5 on the LAN9374),
/// whose speed and duplex settings are ignored otherwise.
pub const CFG_STRAP_OVR_VPHY_DISABLE: u32 = 1 << 31;

// Global switch control (0x03xx).

/// Global switch control 0 (8-bit).
pub const GLB_SW_CONTROL0: u32 = 0x0300;
/// Soft reset of every register except the straps (self-clearing).
pub const GLB_SW_CONTROL0_SOFT_RESET: u8 = 1 << 1;
/// Start the switch.
pub const GLB_SW_CONTROL0_START: u8 = 1 << 0;

/// Switch MAC address (6 bytes), used in pause frames.
pub const GLB_SW_MAC_ADDR: u32 = 0x0302;

/// Lookup engine control 0 (8-bit).
pub const GLB_SW_LUE_CTL0: u32 = 0x0310;
/// Reserved multicast table enable.
pub const LUE_CTL0_RESV_MCAST_EN: u8 = 1 << 2;
/// Lookup engine control 1 (8-bit).
pub const GLB_SW_LUE_CTL1: u32 = 0x0311;
/// Flush the entries matching the MSTP state (ports with learning disabled).
pub const LUE_CTL1_FLUSH_MSTP: u8 = 1 << 4;
/// Flush on link down.
pub const LUE_CTL1_LINK_DOWN_FLUSH: u8 = 1 << 0;
/// Lookup engine control 2 (8-bit).
pub const GLB_SW_LUE_CTL2: u32 = 0x0312;
/// Flush option, bits 3:2: 01 = learned entries only.
pub const LUE_CTL2_FLUSH_DYNAMIC: u8 = 0b01 << 2;
/// Flush option field.
pub const LUE_CTL2_FLUSH_OPTION: u8 = 0b11 << 2;

/// Global MAC control 0 (8-bit).
pub const GLB_SW_MAC_CTL0: u32 = 0x0330;
/// Global MAC control 1 (8-bit).
pub const GLB_SW_MAC_CTL1: u32 = 0x0331;
/// Disable multicast storm protection (only broadcasts count).
pub const MAC_CTL1_MCAST_STORM_DIS: u8 = 1 << 6;
/// Global MAC control 6 (8-bit): MIB flush and freeze.
pub const GLB_SW_MAC_CTL6: u32 = 0x0336;
/// Flush the MIB counters of ports with their flush/freeze enable set (self-clearing).
pub const MAC_CTL6_MIB_FLUSH: u8 = 1 << 7;

// Global address lookup tables (0x04xx).

/// Static and reserved multicast table access control (32-bit).
pub const GLB_STATIC_MCAST_CTL: u32 = 0x041C;
/// Start (self-clearing).
pub const STATIC_CTL_START: u32 = 1 << 7;
/// Direct access.
pub const STATIC_CTL_DIRECT: u32 = 1 << 3;
/// Select the reserved multicast table instead of the static table.
pub const STATIC_CTL_RESV_MCAST: u32 = 1 << 2;
/// Write action.
pub const STATIC_CTL_WRITE: u32 = 0b01;
/// Read action.
pub const STATIC_CTL_READ: u32 = 0b10;
/// Table entry, word 1 (32-bit).
pub const GLB_ALU_TBL_ENT1: u32 = 0x0420;
/// Static entry valid.
pub const ALU_ENT1_STATIC_VALID: u32 = 1 << 31;
/// Table entry, word 2 (32-bit): override and port forwarding mask.
pub const GLB_ALU_TBL_ENT2: u32 = 0x0424;
/// Forward even to ports whose spanning-tree state blocks it.
pub const ALU_ENT2_OVERRIDE: u32 = 1 << 31;
/// Table entry, word 3 (32-bit): FID and MAC bits 47:32.
pub const GLB_ALU_TBL_ENT3: u32 = 0x0428;
/// Table entry, word 4 (32-bit): MAC bits 31:0.
pub const GLB_ALU_TBL_ENT4: u32 = 0x042C;

// Global PTP (0x05xx). The clock ticks 40 ns at 25 MHz, refined by an 8 ns phase.

/// PTP clock control (16-bit). Always read-modify-write it, to keep the
/// enable bits.
pub const PTP_CLK_CTL: u32 = 0x0500;
/// Step the clock by PTP_RTC_NANOSEC/SEC (self-clearing).
pub const PTP_CLK_STEP_ADJ: u16 = 1 << 6;
/// Step direction: 1 = add, 0 = subtract.
pub const PTP_CLK_STEP_DIR: u16 = 1 << 5;
/// Latch the clock into the PTP_RTC registers for reading (self-clearing).
pub const PTP_CLK_READ: u16 = 1 << 4;
/// Load the clock from the PTP_RTC registers (self-clearing).
pub const PTP_CLK_LOAD: u16 = 1 << 3;
/// Continuous rate adjustment enable.
pub const PTP_CLK_ADJ_ENABLE: u16 = 1 << 2;
/// PTP clock enable.
pub const PTP_CLK_ENABLE: u16 = 1 << 1;
/// Real-time clock phase (16-bit): bits 2:0, 8 ns steps within the 40 ns tick.
pub const PTP_RTC_PHASE: u32 = 0x0502;
/// Real-time clock nanoseconds (32-bit).
pub const PTP_RTC_NANOSEC: u32 = 0x0504;
/// Real-time clock seconds (32-bit).
pub const PTP_RTC_SEC: u32 = 0x0508;
/// Rate adjustment (32-bit): bit 31 direction, bits 29:0 in 2^-32 ns per 40 ns tick.
pub const PTP_SUBNANOSEC_RATE: u32 = 0x050C;
/// Rate adjustment direction: 1 = add (the clock runs faster).
pub const PTP_RATE_DIR: u32 = 1 << 31;
/// PTP message configuration 1 (16-bit). Reset value 0x0039.
pub const PTP_MSG_CONF1: u32 = 0x0514;
/// All PTP frames go to the host port only, none to other ports (802.1AS mode).
pub const PTP_802_1AS: u16 = 1 << 7;
/// PTP mode. Every frame from the host must then carry a 4-byte timestamp
/// field before its tail tag, and frames to the host carry receive timestamps.
pub const PTP_ENABLE: u16 = 1 << 6;
/// Detect PTP over IEEE 802.3 (ethertype 0x88F7).
pub const PTP_ETH_ENABLE: u16 = 1 << 5;
/// Detect PTP over UDP/IPv4.
pub const PTP_IPV4_UDP_ENABLE: u16 = 1 << 4;
/// Detect PTP over UDP/IPv6.
pub const PTP_IPV6_UDP_ENABLE: u16 = 1 << 3;
/// Peer-to-peer (vs end-to-end) transparent clock.
pub const PTP_TC_P2P: u16 = 1 << 2;
/// The host port is a leader ordinary clock.
pub const PTP_MASTER: u16 = 1 << 1;
/// One-step (reset default); 0 = two-step.
pub const PTP_1STEP: u16 = 1 << 0;

// Global virtual PHY (0x07xx).

/// Indirect PHY access: target address (16-bit).
pub const VPHY_IND_ADDR: u32 = 0x075C;
/// Indirect PHY access: data (16-bit).
pub const VPHY_IND_DATA: u32 = 0x0760;
/// Indirect PHY access: control (16-bit).
pub const VPHY_IND_CTRL: u32 = 0x0768;
/// Indirect access is a write.
pub const VPHY_IND_WRITE: u16 = 1 << 1;
/// Start; clears when done.
pub const VPHY_IND_BUSY: u16 = 1 << 0;
/// Virtual PHY special control (16-bit).
pub const VPHY_SPECIAL_CTRL: u32 = 0x077C;
/// Route the T1 PHYs to the MDIO pins ("side MDIO").
pub const VPHY_MDIO_INT_EN: u16 = 1 << 13;
/// Indirect T1 PHY access over SPI.
pub const VPHY_SPI_INDIRECT_EN: u16 = 1 << 12;

// Port operation (0xN0xx).

/// Port interrupt status (8-bit, read-only).
pub const P_INT_STS: u16 = 0x001B;
/// Port interrupt mask (8-bit), 1 = masked.
pub const P_INT_MSK: u16 = 0x001F;
/// PTP interrupt.
pub const P_INT_PTP: u8 = 1 << 2;
/// PHY interrupt.
pub const P_INT_PHY: u8 = 1 << 1;
/// Port operation control 0 (8-bit).
pub const P_OP_CTL0: u16 = 0x0020;
/// Tail tag enable: makes the port the host port.
pub const P_OP_CTL0_TAIL_TAG: u8 = 1 << 2;
/// Port operation status (8-bit, read-only): bits 4:3 speed, bit 2 duplex.
pub const P_OP_STS: u16 = 0x0030;

// T1 PHY registers (0xN1xx): clause-22 register n at 0x100 + 4n.

/// The indirect-access address of T1 PHY register `reg` of `port`.
pub const fn t1_phy(port: Port, reg: u8) -> u16 {
    ((((port.0 as u32) + 1) << 12) | 0x0100 | ((reg as u32) << 2)) as u16
}

// Port xMII (0xN3xx).

/// xMII control 0 (8-bit): bit 6 full duplex, bit 5 TX flow control, bit 4 100
/// (vs 10) Mbit/s, bit 3 RX flow control.
pub const P_XMII_CTL0: u16 = 0x0300;
/// Full duplex.
pub const XMII_CTL0_FULL_DUPLEX: u8 = 1 << 6;
/// Send pause frames.
pub const XMII_CTL0_TX_FLOW: u8 = 1 << 5;
/// 100 Mbit/s rather than 10 (when not 1000).
pub const XMII_CTL0_100: u8 = 1 << 4;
/// Honour received pause frames.
pub const XMII_CTL0_RX_FLOW: u8 = 1 << 3;
/// xMII control 1 (8-bit).
pub const P_XMII_CTL1: u16 = 0x0301;
/// 10/100 Mbit/s (per CTL0) rather than 1000.
pub const XMII_CTL1_NOT_1000: u8 = 1 << 6;
/// RGMII ingress (RX clock) internal delay.
pub const XMII_CTL1_RGMII_ID_IG: u8 = 1 << 4;
/// RGMII egress (TX clock) internal delay.
pub const XMII_CTL1_RGMII_ID_EG: u8 = 1 << 3;
/// RMII: the reference clock is an input (0 = the switch drives it).
pub const XMII_CTL1_RMII_REFCLK_IN: u8 = 1 << 2;
/// Interface select, bits 1:0: 00 RGMII, 01 RMII, 1x MII.
pub const XMII_CTL1_SEL: u8 = 0b11;
/// RGMII.
pub const XMII_CTL1_SEL_RGMII: u8 = 0b00;
/// RMII.
pub const XMII_CTL1_SEL_RMII: u8 = 0b01;
/// xMII control 3 (8-bit, read-only): RGMII in-band status.
pub const P_XMII_CTL3: u16 = 0x0303;
/// RX DLL control (16-bit): bit 15 reset, bits 13:7 tune.
pub const P_XMII_CTL4: u16 = 0x0304;
/// TX DLL control (16-bit): bit 15 reset, bits 13:7 tune.
pub const P_XMII_CTL5: u16 = 0x0306;

// Port MAC (0xN4xx).

/// MAC control 0 (8-bit).
pub const P_MAC_CTL0: u16 = 0x0400;
/// Check the 802.3 length field against the frame size. Breaks tail-tagged LLC frames.
pub const MAC_CTL0_FR_LEN_CHK: u8 = 1 << 2;
/// Broadcast storm protection.
pub const MAC_CTL0_BCAST_STORM: u8 = 1 << 1;
/// Frames up to MAX_FR_SIZE (instead of 1518/1522).
pub const MAC_CTL0_JUMBO: u8 = 1 << 0;
/// Maximum frame size (16-bit), FCS and tags included.
pub const P_MAX_FR_SIZE: u16 = 0x0404;

// Port MIB (0xN5xx).

/// MIB control and status (32-bit).
pub const P_MIB_CTL: u16 = 0x0500;
/// Read the counter (self-clearing when done).
pub const MIB_CTL_READ: u32 = 1 << 25;
/// Enable the global flush/freeze for this port.
pub const MIB_CTL_FLUSH_FREEZE: u32 = 1 << 24;
/// MIB data (32-bit): counter bits 31:0.
pub const P_MIB_DATA: u16 = 0x0504;

// Port membership and lookup (0xNAxx, 0xNBxx).

/// Port VLAN membership (32-bit): bits 7:0 = ports frames from this port may go to.
pub const P_MEMBERSHIP: u16 = 0x0A04;
/// Spanning-tree state for the current MSTP pointer (8-bit).
pub const P_STP_STATE: u16 = 0x0B04;
/// Transmit enable.
pub const STP_TX_EN: u8 = 1 << 2;
/// Receive enable.
pub const STP_RX_EN: u8 = 1 << 1;
/// Learning disable.
pub const STP_LEARN_DIS: u8 = 1 << 0;

// Port PTP (0xNCxx). Timestamps are partial: bits 31:30 seconds mod 4,
// bits 29:0 nanoseconds.

/// RX latency (16-bit, ns), wire to timestamp point. Default 415.
pub const P_PTP_RX_LATENCY: u16 = 0x0C00;
/// TX latency (16-bit, ns), timestamp point to wire. Default 45.
pub const P_PTP_TX_LATENCY: u16 = 0x0C02;
/// Link asymmetry (16-bit): bit 15 negative, bits 14:0 ns.
pub const P_PTP_ASYM: u16 = 0x0C04;
/// TX timestamp of the last Delay_Req or Pdelay_Req (32-bit).
pub const P_PTP_XDELAY_TS: u16 = 0x0C08;
/// TX timestamp of the last Sync (32-bit).
pub const P_PTP_SYNC_TS: u16 = 0x0C0C;
/// TX timestamp of the last Pdelay_Resp (32-bit).
pub const P_PTP_PDRESP_TS: u16 = 0x0C10;
/// TX timestamp interrupt status (16-bit, W1C).
pub const P_PTP_TS_INT_STS: u16 = 0x0C14;
/// TX timestamp interrupt enable (16-bit).
pub const P_PTP_TS_INT_EN: u16 = 0x0C16;
/// Sync timestamp ready.
pub const PTP_TS_INT_SYNC: u16 = 1 << 15;
/// Delay_Req/Pdelay_Req timestamp ready.
pub const PTP_TS_INT_XDELAY: u16 = 1 << 14;
/// Pdelay_Resp timestamp ready.
pub const PTP_TS_INT_PDRESP: u16 = 1 << 13;
