//! The switch, over SPI.

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::SpiDevice;

use crate::fmt::{debug, info};
use crate::spi::Spi;
use crate::stp::{StpState, membership};
use crate::{Duplex, Error, Port, Speed, reg};

/// Polls of a self-clearing busy bit before giving up. Each poll is a SPI
/// transaction of a few microseconds.
const BUSY_POLLS: usize = 1000;

/// The chip, from its ID registers.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChipId {
    /// `0x70`–`0x74` for LAN9370–LAN9374.
    pub chip: u8,
    /// 0 for A0, 1 for B0.
    pub revision: u8,
}

/// How an xMII port is wired.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Xmii {
    /// RMII, 100 Mbit/s full duplex.
    Rmii {
        /// The switch drives the 50 MHz reference clock (REFCLKO) rather than
        /// receiving it.
        drive_refclk: bool,
    },
    /// RGMII. Each direction's 2 ns clock delay must be added exactly once,
    /// by the switch, the PHY or the board.
    Rgmii {
        /// Delay the transmit clock (switch to PHY) in the switch, with this
        /// DLL tune code, or `None` for no delay.
        tx_delay: Option<u8>,
        /// Delay the receive clock (PHY to switch) in the switch, with this
        /// DLL tune code, or `None` for no delay.
        rx_delay: Option<u8>,
    },
}

/// Linux's characterized "2 ns" DLL tune codes (`lan937x_reg.h`).
pub mod delay_codes {
    /// RGMII 1 (port 6) transmit.
    pub const RGMII1_TX_2NS: u8 = 0x02;
    /// RGMII 1 (port 6) receive.
    pub const RGMII1_RX_2NS: u8 = 0x1B;
    /// RGMII 2 (port 5) transmit.
    pub const RGMII2_TX_2NS: u8 = 0x00;
    /// RGMII 2 (port 5) receive.
    pub const RGMII2_RX_2NS: u8 = 0x14;
}

/// Switch configuration for [`Lan937x::init`].
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone)]
pub struct Config {
    /// The number of ports (8 on the LAN9374).
    pub ports: u8,
    /// The host port: the one with tail tagging. Must be an xMII port.
    pub host: Port,
    /// How the host port is wired.
    pub host_xmii: Xmii,
    /// Another xMII port to configure, typically an uplink to an external PHY.
    /// Its speed follows the PHY's link, see [`Lan937x::set_xmii_link`].
    pub other_xmii: Option<(Port, Xmii)>,
    /// The spanning-tree state user ports start in.
    pub initial_state: StpState,
    /// The switch's MAC address, the source of pause frames.
    pub mac: [u8; 6],
}

/// Registers of the per-port MIB counters, by index.
#[allow(missing_docs)]
pub mod mib {
    pub const RX_CRC_ERROR: u8 = 0x06;
    pub const RX_BROADCAST: u8 = 0x0A;
    pub const RX_MULTICAST: u8 = 0x0B;
    pub const RX_UNICAST: u8 = 0x0C;
    pub const TX_BROADCAST: u8 = 0x18;
    pub const TX_MULTICAST: u8 = 0x19;
    pub const TX_UNICAST: u8 = 0x1A;
    pub const RX_BYTES: u8 = 0x80;
    pub const TX_BYTES: u8 = 0x81;
    pub const RX_DROPS: u8 = 0x82;
    pub const TX_DROPS: u8 = 0x83;
}

/// The reserved multicast groups (`01:80:c2:00:00:xx`), by the address used to
/// select each one. Editing one address edits its whole group.
mod resv_group {
    /// `-00`: spanning tree.
    pub const STP: u8 = 0x00;
    /// `-03`: 802.1X.
    pub const DOT1X: u8 = 0x03;
    /// `-02`, `-04`..`-0F`: slow protocols, LLDP, PTP peer delay.
    pub const LINK_LOCAL: u8 = 0x02;
    /// `-20`: GMRP.
    pub const GMRP: u8 = 0x20;
    /// `-21`: GVRP.
    pub const GVRP: u8 = 0x21;
    /// `-11`..`-1F`, `-22`..`-2F`.
    pub const OTHER: u8 = 0x11;
}

/// A LAN937x switch on SPI.
pub struct Lan937x<SPI, D> {
    spi: Spi<SPI>,
    delay: D,
}

impl<SPI: SpiDevice, D: DelayNs> Lan937x<SPI, D> {
    /// A switch on `spi`, which must be in SPI mode 0 at no more than 50 MHz.
    pub fn new(spi: SPI, delay: D) -> Self {
        Self {
            spi: Spi::new(spi),
            delay,
        }
    }

    /// Raw register access.
    pub fn regs(&mut self) -> &mut Spi<SPI> {
        &mut self.spi
    }

    /// Read and check the chip ID. Fails if it isn't a LAN937x.
    pub async fn identify(&mut self) -> Result<ChipId, Error<SPI::Error>> {
        let id = self.spi.read32(reg::CHIP_ID).await?;
        let [zero, family, chip, rev] = id.to_be_bytes();
        if zero != 0 || family != 0x93 || !(0x70..=0x74).contains(&chip) {
            return Err(Error::UnknownChip(id));
        }
        Ok(ChipId {
            chip,
            revision: rev >> 4,
        })
    }

    /// Wait for the switch to finish initializing its memories after reset.
    pub async fn wait_ready(&mut self) -> Result<(), Error<SPI::Error>> {
        for _ in 0..100 {
            if self.spi.read8(reg::GLB_CONTROL0).await? & reg::GLB_CONTROL0_SW_INI_DONE != 0 {
                return Ok(());
            }
            self.delay.delay_ms(1).await;
        }
        Err(Error::Timeout)
    }

    /// Configure the switch, and start it.
    ///
    /// Afterwards the host port is tail tagged in PTP mode (6-byte tails from
    /// the host), user ports are in `config.initial_state`, port-local
    /// multicast goes to the host only, and every interrupt is masked.
    pub async fn init(&mut self, config: &Config) -> Result<(), Error<SPI::Error>> {
        let id = self.identify().await?;
        info!("lan937x: chip {:x} rev {}", id.chip, id.revision);
        self.wait_ready().await?;

        // Interrupts: everything masked, and POR_READY (which holds IRQ_N
        // asserted from power-on and can't be masked) cleared.
        self.spi.write32(reg::GLB_INT_MSK, 0xfe00_0000).await?;
        self.spi.write32(reg::GLB_INT_STS, reg::GLB_INT_POR_READY).await?;
        self.spi.write32(reg::GLB_PINT_MSK, 0xff).await?;

        // Hold forwarding off while configuring.
        self.spi
            .modify8(reg::GLB_SW_CONTROL0, reg::GLB_SW_CONTROL0_START, 0)
            .await?;

        // Single chip: cascade mode changes the tail tag format.
        self.spi
            .modify32(reg::GLB_CASCADE_MODE_CTL, reg::GLB_CASCADE_MODE_EN, 0)
            .await?;
        // The virtual PHY would otherwise override its port's speed settings.
        self.spi
            .modify32(reg::CFG_STRAP_OVR, 0, reg::CFG_STRAP_OVR_VPHY_DISABLE)
            .await?;
        // T1 PHY registers: unblock them, and reach them through the indirect
        // SPI window, not the MDIO pins.
        self.spi
            .modify8(reg::GLB_CONTROL0, reg::GLB_CONTROL0_APB_PHY_REG_BLK, 0)
            .await?;
        self.spi
            .modify16(
                reg::VPHY_SPECIAL_CTRL,
                reg::VPHY_MDIO_INT_EN | reg::VPHY_SPI_INDIRECT_EN,
                reg::VPHY_SPI_INDIRECT_EN,
            )
            .await?;
        // CLK125 and CLKO_25M are only used as straps.
        self.spi.write8(reg::GLB_OUT_CLK_CTL, 0x03).await?;
        self.spi.write_bytes(reg::GLB_SW_MAC_ADDR, &config.mac).await?;

        self.init_host_port(config.host, config.host_xmii).await?;
        if let Some((port, xmii)) = config.other_xmii {
            self.configure_xmii(port, xmii).await?;
        }

        self.init_reserved_multicast(config.host).await?;
        // PTP messages over 802.3 to their non-peer-delay group go to the host only.
        self.write_static_entry(0, [0x01, 0x1b, 0x19, 0, 0, 0], 1 << config.host.0, true)
            .await?;

        // PTP mode, so frames carry receive timestamps and the host's frames
        // the timestamp field. All PTP frames go to the host only (802.1AS
        // mode); the host runs a boundary clock. Two-step.
        self.spi
            .write16(
                reg::PTP_MSG_CONF1,
                reg::PTP_802_1AS | reg::PTP_ENABLE | reg::PTP_ETH_ENABLE | reg::PTP_TC_P2P,
            )
            .await?;
        self.spi.modify16(reg::PTP_CLK_CTL, 0, reg::PTP_CLK_ENABLE).await?;

        self.flush_mib(config.ports).await?;

        // Flush a port's learned addresses when its link goes down.
        self.spi
            .modify8(reg::GLB_SW_LUE_CTL1, 0, reg::LUE_CTL1_LINK_DOWN_FLUSH)
            .await?;

        // Port states: the host forwards and learns; user ports start in the
        // configured state.
        let mut forwarding = 0;
        for port in (0..config.ports).map(Port) {
            let state = if port == config.host {
                StpState::Forwarding
            } else {
                config.initial_state
            };
            if state == StpState::Forwarding && port != config.host {
                forwarding |= 1 << port.0;
            }
            self.write_stp_state(port, state).await?;
        }
        self.write_membership(config.ports, config.host, forwarding).await?;

        self.spi
            .modify8(reg::GLB_SW_CONTROL0, 0, reg::GLB_SW_CONTROL0_START)
            .await?;
        debug!("lan937x: started");
        Ok(())
    }

    async fn init_host_port(&mut self, port: Port, xmii: Xmii) -> Result<(), Error<SPI::Error>> {
        self.configure_xmii(port, xmii).await?;
        if let Xmii::Rmii { .. } = xmii {
            self.set_xmii_link(port, Speed::Mbps100, Duplex::Full, false, false)
                .await?;
        }
        self.spi
            .modify8(reg::port(port, reg::P_OP_CTL0), 0, reg::P_OP_CTL0_TAIL_TAG)
            .await?;
        // Tail tags make frames longer than the standard limits, and break the
        // 802.3 length check of LLC frames.
        self.spi
            .modify8(
                reg::port(port, reg::P_MAC_CTL0),
                reg::MAC_CTL0_FR_LEN_CHK,
                reg::MAC_CTL0_JUMBO,
            )
            .await?;
        self.spi.write16(reg::port(port, reg::P_MAX_FR_SIZE), 2000).await?;
        Ok(())
    }

    /// Set an xMII port's interface mode and clock delays. Doesn't touch its
    /// speed, duplex or flow control.
    pub async fn configure_xmii(&mut self, port: Port, xmii: Xmii) -> Result<(), Error<SPI::Error>> {
        let ctl1 = reg::port(port, reg::P_XMII_CTL1);
        let mask = reg::XMII_CTL1_SEL
            | reg::XMII_CTL1_RMII_REFCLK_IN
            | reg::XMII_CTL1_RGMII_ID_EG
            | reg::XMII_CTL1_RGMII_ID_IG;
        match xmii {
            Xmii::Rmii { drive_refclk } => {
                let mut value = reg::XMII_CTL1_SEL_RMII | reg::XMII_CTL1_NOT_1000;
                if !drive_refclk {
                    value |= reg::XMII_CTL1_RMII_REFCLK_IN;
                }
                self.spi.modify8(ctl1, mask | reg::XMII_CTL1_NOT_1000, value).await?;
            }
            Xmii::Rgmii { tx_delay, rx_delay } => {
                if let Some(code) = tx_delay {
                    self.tune_dll(reg::port(port, reg::P_XMII_CTL5), code).await?;
                }
                if let Some(code) = rx_delay {
                    self.tune_dll(reg::port(port, reg::P_XMII_CTL4), code).await?;
                }
                let mut value = reg::XMII_CTL1_SEL_RGMII;
                if tx_delay.is_some() {
                    value |= reg::XMII_CTL1_RGMII_ID_EG;
                }
                if rx_delay.is_some() {
                    value |= reg::XMII_CTL1_RGMII_ID_IG;
                }
                self.spi.modify8(ctl1, mask, value).await?;
            }
        }
        Ok(())
    }

    /// Set a DLL's tune code, then reset the DLL so it takes effect (Linux
    /// `lan937x_set_tune_adj`).
    async fn tune_dll(&mut self, addr: u32, code: u8) -> Result<(), Error<SPI::Error>> {
        const TUNE: u16 = 0x7f << 7;
        const RESET: u16 = 1 << 15;
        let value = (self.spi.read16(addr).await? & !TUNE & !RESET) | ((code as u16) << 7);
        self.spi.write16(addr, value).await?;
        self.spi.write16(addr, value | RESET).await
    }

    /// Set an xMII port's MAC to a link's speed, duplex and flow control, for
    /// example after its PHY's link came up.
    pub async fn set_xmii_link(
        &mut self,
        port: Port,
        speed: Speed,
        duplex: Duplex,
        tx_pause: bool,
        rx_pause: bool,
    ) -> Result<(), Error<SPI::Error>> {
        let mut ctl0 = 0;
        if duplex == Duplex::Full {
            ctl0 |= reg::XMII_CTL0_FULL_DUPLEX;
        }
        if speed == Speed::Mbps100 {
            ctl0 |= reg::XMII_CTL0_100;
        }
        if tx_pause {
            ctl0 |= reg::XMII_CTL0_TX_FLOW;
        }
        if rx_pause {
            ctl0 |= reg::XMII_CTL0_RX_FLOW;
        }
        self.spi
            .modify8(
                reg::port(port, reg::P_XMII_CTL0),
                reg::XMII_CTL0_FULL_DUPLEX | reg::XMII_CTL0_100 | reg::XMII_CTL0_TX_FLOW | reg::XMII_CTL0_RX_FLOW,
                ctl0,
            )
            .await?;
        let not_1000 = if speed == Speed::Mbps1000 {
            0
        } else {
            reg::XMII_CTL1_NOT_1000
        };
        self.spi
            .modify8(reg::port(port, reg::P_XMII_CTL1), reg::XMII_CTL1_NOT_1000, not_1000)
            .await?;
        Ok(())
    }

    /// Point the reserved multicast table at the host: spanning tree, 802.1X
    /// and the other link-local groups (LLDP, PTP peer delay, slow protocols)
    /// to the host only, from every port whatever its state; the GARP groups
    /// and the rest to every port but the host.
    async fn init_reserved_multicast(&mut self, host: Port) -> Result<(), Error<SPI::Error>> {
        let host_mask = 1u8 << host.0;
        for group in [resv_group::STP, resv_group::DOT1X, resv_group::LINK_LOCAL] {
            self.write_reserved_multicast(group, host_mask, true).await?;
        }
        for group in [resv_group::GMRP, resv_group::GVRP, resv_group::OTHER] {
            self.write_reserved_multicast(group, !host_mask, false).await?;
        }
        self.spi
            .modify8(reg::GLB_SW_LUE_CTL0, 0, reg::LUE_CTL0_RESV_MCAST_EN)
            .await?;
        Ok(())
    }

    async fn static_table_command(&mut self, command: u32) -> Result<(), Error<SPI::Error>> {
        self.spi
            .write32(reg::GLB_STATIC_MCAST_CTL, command | reg::STATIC_CTL_START)
            .await?;
        for _ in 0..BUSY_POLLS {
            if self.spi.read32(reg::GLB_STATIC_MCAST_CTL).await? & reg::STATIC_CTL_START == 0 {
                return Ok(());
            }
        }
        Err(Error::Timeout)
    }

    /// Set where frames to reserved multicast group `group` (selected by the
    /// last byte of any of its addresses) go.
    pub async fn write_reserved_multicast(
        &mut self,
        group: u8,
        ports: u8,
        override_port_state: bool,
    ) -> Result<(), Error<SPI::Error>> {
        let select = ((group as u32) << 8) | reg::STATIC_CTL_DIRECT | reg::STATIC_CTL_RESV_MCAST;
        self.static_table_command(select | reg::STATIC_CTL_READ).await?;
        let old = self.spi.read32(reg::GLB_ALU_TBL_ENT2).await?;
        let mut value = (old & !0xff & !reg::ALU_ENT2_OVERRIDE) | ports as u32;
        if override_port_state {
            value |= reg::ALU_ENT2_OVERRIDE;
        }
        self.spi.write32(reg::GLB_ALU_TBL_ENT2, value).await?;
        self.static_table_command(select | reg::STATIC_CTL_WRITE).await
    }

    /// Write entry `index` (0..256) of the static address table: frames to
    /// `mac` go to `ports`.
    pub async fn write_static_entry(
        &mut self,
        index: u8,
        mac: [u8; 6],
        ports: u8,
        override_port_state: bool,
    ) -> Result<(), Error<SPI::Error>> {
        let override_bit = if override_port_state { reg::ALU_ENT2_OVERRIDE } else { 0 };
        self.spi
            .write32(reg::GLB_ALU_TBL_ENT1, reg::ALU_ENT1_STATIC_VALID)
            .await?;
        self.spi
            .write32(reg::GLB_ALU_TBL_ENT2, override_bit | ports as u32)
            .await?;
        self.spi
            .write32(reg::GLB_ALU_TBL_ENT3, u16::from_be_bytes([mac[0], mac[1]]) as u32)
            .await?;
        self.spi
            .write32(
                reg::GLB_ALU_TBL_ENT4,
                u32::from_be_bytes([mac[2], mac[3], mac[4], mac[5]]),
            )
            .await?;
        // Linux omits the write action here (ksz9477_mdb_add), which is a no-op on LAN937x.
        self.static_table_command(((index as u32) << 8) | reg::STATIC_CTL_WRITE)
            .await
    }

    /// Write a port's spanning-tree state register. Membership is separate,
    /// see [`write_membership`](Self::write_membership).
    pub async fn write_stp_state(&mut self, port: Port, state: StpState) -> Result<(), Error<SPI::Error>> {
        self.spi
            .modify8(
                reg::port(port, reg::P_STP_STATE),
                reg::STP_TX_EN | reg::STP_RX_EN | reg::STP_LEARN_DIS,
                state.register(),
            )
            .await?;
        Ok(())
    }

    /// Write every port's membership for the set of forwarding user ports
    /// (bit n = port n), as [`membership`] computes it.
    pub async fn write_membership(&mut self, ports: u8, host: Port, forwarding: u8) -> Result<(), Error<SPI::Error>> {
        let masks = membership(ports, host, forwarding);
        for port in (0..ports).map(Port) {
            self.spi
                .write32(reg::port(port, reg::P_MEMBERSHIP), masks[port.0 as usize] as u32)
                .await?;
        }
        Ok(())
    }

    /// Flush the addresses the switch learned on `port` (Linux
    /// `ksz9477_flush_dyn_mac_table`).
    pub async fn flush_port(&mut self, port: Port) -> Result<(), Error<SPI::Error>> {
        self.spi
            .modify8(
                reg::GLB_SW_LUE_CTL2,
                reg::LUE_CTL2_FLUSH_OPTION,
                reg::LUE_CTL2_FLUSH_DYNAMIC,
            )
            .await?;
        // Entries of ports with learning disabled are flushed.
        let stp = reg::port(port, reg::P_STP_STATE);
        let state = self.spi.read8(stp).await?;
        if state & reg::STP_LEARN_DIS == 0 {
            self.spi.write8(stp, state | reg::STP_LEARN_DIS).await?;
        }
        self.spi
            .modify8(reg::GLB_SW_LUE_CTL1, 0, reg::LUE_CTL1_FLUSH_MSTP)
            .await?;
        let mut done = false;
        for _ in 0..BUSY_POLLS {
            if self.spi.read8(reg::GLB_SW_LUE_CTL1).await? & reg::LUE_CTL1_FLUSH_MSTP == 0 {
                done = true;
                break;
            }
        }
        if state & reg::STP_LEARN_DIS == 0 {
            self.spi.write8(stp, state).await?;
        }
        if done { Ok(()) } else { Err(Error::Timeout) }
    }

    async fn flush_mib(&mut self, ports: u8) -> Result<(), Error<SPI::Error>> {
        for port in (0..ports).map(Port) {
            self.spi
                .write32(reg::port(port, reg::P_MIB_CTL), reg::MIB_CTL_FLUSH_FREEZE)
                .await?;
        }
        self.spi.write8(reg::GLB_SW_MAC_CTL6, reg::MAC_CTL6_MIB_FLUSH).await?;
        for port in (0..ports).map(Port) {
            self.spi.write32(reg::port(port, reg::P_MIB_CTL), 0).await?;
        }
        Ok(())
    }

    /// Read MIB counter `index` of `port` (see [`mib`]). Counters clear when
    /// read, so the caller accumulates.
    pub async fn read_mib(&mut self, port: Port, index: u8) -> Result<u64, Error<SPI::Error>> {
        let ctl = reg::port(port, reg::P_MIB_CTL);
        self.spi
            .write32(ctl, ((index as u32) << 16) | reg::MIB_CTL_READ)
            .await?;
        for _ in 0..BUSY_POLLS {
            let status = self.spi.read32(ctl).await?;
            if status & reg::MIB_CTL_READ == 0 {
                let low = self.spi.read32(reg::port(port, reg::P_MIB_DATA)).await?;
                return Ok((((status & 0xf) as u64) << 32) | low as u64);
            }
        }
        Err(Error::Timeout)
    }

    /// The ports with a pending interrupt (bit n = port n).
    pub async fn pending_ports(&mut self) -> Result<u8, Error<SPI::Error>> {
        Ok(self.spi.read32(reg::GLB_PINT_STS).await? as u8)
    }

    /// A port's pending interrupt sources ([`reg::P_INT_PHY`], [`reg::P_INT_PTP`]).
    pub async fn port_interrupts(&mut self, port: Port) -> Result<u8, Error<SPI::Error>> {
        self.spi.read8(reg::port(port, reg::P_INT_STS)).await
    }

    /// Unmask interrupt sources: `sources` of each port in `ports`.
    pub async fn enable_port_interrupts(&mut self, ports: u8, sources: u8) -> Result<(), Error<SPI::Error>> {
        for port in (0..8).map(Port).filter(|p| ports & (1 << p.0) != 0) {
            self.spi.modify8(reg::port(port, reg::P_INT_MSK), sources, 0).await?;
        }
        self.spi.modify32(reg::GLB_PINT_MSK, ports as u32, 0).await?;
        Ok(())
    }

    /// Wait for an indirect PHY access to finish. Usually a few microseconds,
    /// but a PHY that is resetting (or running its power-on initialization)
    /// holds the access for tens of milliseconds.
    async fn wait_indirect(&mut self) -> Result<(), Error<SPI::Error>> {
        for poll in 0..2000 {
            if self.spi.read16(reg::VPHY_IND_CTRL).await? & reg::VPHY_IND_BUSY == 0 {
                return Ok(());
            }
            if poll >= 20 {
                self.delay.delay_us(100).await;
            }
        }
        Err(Error::Timeout)
    }

    /// Read clause-22 register `reg` of the internal PHY of `port`, through
    /// the indirect SPI window.
    pub async fn phy_read(&mut self, port: Port, phy_reg: u8) -> Result<u16, Error<SPI::Error>> {
        self.spi.write16(reg::VPHY_IND_ADDR, reg::t1_phy(port, phy_reg)).await?;
        self.spi.write16(reg::VPHY_IND_CTRL, reg::VPHY_IND_BUSY).await?;
        self.wait_indirect().await?;
        self.spi.read16(reg::VPHY_IND_DATA).await
    }

    /// Write clause-22 register `reg` of the internal PHY of `port`, through
    /// the indirect SPI window.
    pub async fn phy_write(&mut self, port: Port, phy_reg: u8, value: u16) -> Result<(), Error<SPI::Error>> {
        self.spi.write16(reg::VPHY_IND_ADDR, reg::t1_phy(port, phy_reg)).await?;
        self.spi.write16(reg::VPHY_IND_DATA, value).await?;
        self.spi
            .write16(reg::VPHY_IND_CTRL, reg::VPHY_IND_WRITE | reg::VPHY_IND_BUSY)
            .await?;
        self.wait_indirect().await
    }

    /// Wait `us` microseconds.
    pub async fn delay_us(&mut self, us: u32) {
        self.delay.delay_us(us).await
    }
}
