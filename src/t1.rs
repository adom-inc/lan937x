//! The integrated 100BASE-T1 PHYs.
//!
//! Each T1 port has a PHY with the usual clause-22 registers plus banked
//! extended registers behind registers 0x14..0x16. They're reached through
//! the switch's indirect SPI window ([`Lan937x::phy_read`]).
//!
//! The init sequence is Linux's (`microchip_t1.c`, `lan87xx_phy_init`), which
//! Microchip contributed. AN4005 documents some of the registers it writes and
//! agrees with it wherever it does; the rest are undocumented DSP settings.

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::SpiDevice;

use crate::{Error, Lan937x, Port};

/// Basic control.
pub const BMCR: u8 = 0x00;
/// Soft reset (self-clearing). Set it alone.
pub const BMCR_RESET: u16 = 1 << 15;
/// Loopback.
pub const BMCR_LOOPBACK: u16 = 1 << 14;
/// Basic status.
pub const BMSR: u8 = 0x01;
/// PHY identifier 1.
pub const PHYID1: u8 = 0x02;
/// PHY identifier 2.
pub const PHYID2: u8 = 0x03;
/// Master/slave control.
pub const MS_CTL: u8 = 0x09;
/// Manual master/slave configuration enable. Always set.
pub const MS_CTL_MANUAL: u16 = 1 << 12;
/// Master (leader).
pub const MS_CTL_MASTER: u16 = 1 << 11;
/// Master/slave status.
pub const MS_STS: u8 = 0x0A;
/// MDIO control 2.
pub const MDIO_CTL2: u8 = 0x10;
/// Mode status.
pub const MODE_STS: u8 = 0x11;
/// Link up: local and remote receivers OK and the descrambler locked.
pub const MODE_STS_LINK: u16 = 1 << 0;
/// Extended register access control.
pub const EXT_REG_CTL: u8 = 0x14;
const EXT_REG_RD: u16 = 1 << 12;
const EXT_REG_WR: u16 = 1 << 11;
/// Extended register read data.
pub const EXT_REG_RD_DATA: u8 = 0x15;
/// Extended register write data.
pub const EXT_REG_WR_DATA: u8 = 0x16;
/// Interrupt source (read to clear).
pub const INT_SOURCE: u8 = 0x18;
/// Interrupt mask (1 = enabled).
pub const INT_MASK: u8 = 0x19;
/// Link down interrupt.
pub const INT_LINK_DOWN: u16 = 1 << 1;
/// Link up interrupt.
pub const INT_LINK_UP: u16 = 1 << 2;
/// Power-down control.
pub const POWER_DOWN_CTL: u8 = 0x1A;

/// Misc bank: interrupt 2 source (read to clear).
const MISC_INT2_SOURCE: u8 = 0x08;
/// Misc bank: interrupt 2 mask (1 = enabled).
const MISC_INT2_MASK: u8 = 0x09;
/// Communication ready (link up).
const INT2_COMM_READY: u16 = 1 << 10;
/// DSP bank: signal quality.
const DSP_DCQ_SQI: u8 = 0xC3;

/// The PHY ID of the integrated T1 PHYs, revision bits masked.
pub const PHY_ID: u32 = 0x0007_c180;
/// The mask for [`PHY_ID`].
pub const PHY_ID_MASK: u32 = 0xffff_fff0;

/// A register bank.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Bank {
    /// Plain clause-22 registers.
    Smi = 0,
    /// Miscellaneous.
    Misc = 1,
    /// PCS.
    Pcs = 2,
    /// Analog front end.
    Afe = 3,
    /// DSP.
    Dsp = 4,
}

/// 100BASE-T1 has no autonegotiation: the two ends of a link must be
/// configured opposite.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Role {
    /// Leader. Provides the clock.
    Master,
    /// Follower. The PHY's reset default.
    #[default]
    Slave,
}

#[derive(Debug, Clone, Copy)]
enum Op {
    /// Write a register.
    W(Bank, u8, u16),
    /// Read a register and discard the value (to clear read-to-clear status).
    R(Bank, u8),
    /// Poll a clause-22 register until `value & mask == 0`.
    Poll(u8, u16),
}

use Bank::*;
use Op::*;

/// Linux `hw_init`: forces the transmitter off and holds the hardware init.
const HW_INIT: &[Op] = &[W(Afe, 0x0B, 0x002D), W(Smi, 0x1A, 0x0308)];

/// Linux `slave_init`: equalizer freeze settings for followers.
const SLAVE_INIT: &[Op] = &[
    W(Dsp, 0x69, 0x0002),
    W(Dsp, 0x6A, 0x0002),
    W(Dsp, 0x6B, 0x0002),
    W(Dsp, 0x6C, 0x0002),
    W(Dsp, 0x6D, 0x0002),
    W(Dsp, 0x6E, 0x0002),
];

/// Linux `phy_init`, in order.
const PHY_INIT: &[Op] = &[
    W(Dsp, 0x18, 0x0D53),
    W(Dsp, 0x05, 0x0AB2),
    W(Dsp, 0x06, 0x0AB3),
    W(Dsp, 0x1A, 0x0AEA),
    W(Dsp, 0x1B, 0x0AEB),
    W(Dsp, 0x1C, 0x0AEB),
    W(Dsp, 0x02, 0x1C00),
    // TX FIR filter.
    W(Dsp, 0x55, 0x1000),
    W(Dsp, 0x55, 0x1861),
    W(Dsp, 0x55, 0x1061),
    W(Dsp, 0x55, 0x1922),
    W(Dsp, 0x55, 0x1122),
    W(Dsp, 0x55, 0x1983),
    W(Dsp, 0x55, 0x1183),
    W(Dsp, 0x55, 0x1944),
    W(Dsp, 0x55, 0x1144),
    W(Dsp, 0x55, 0x18C5),
    W(Dsp, 0x55, 0x10C5),
    W(Dsp, 0x55, 0x1846),
    W(Dsp, 0x55, 0x1046),
    W(Dsp, 0x55, 0x1807),
    W(Dsp, 0x55, 0x1007),
    W(Dsp, 0x55, 0x1808),
    W(Dsp, 0x55, 0x1008),
    W(Dsp, 0x55, 0x1809),
    W(Dsp, 0x55, 0x1009),
    W(Dsp, 0x55, 0x180A),
    W(Dsp, 0x55, 0x100A),
    W(Dsp, 0x55, 0x180B),
    W(Dsp, 0x55, 0x100B),
    W(Dsp, 0x55, 0x180C),
    W(Dsp, 0x55, 0x100C),
    W(Dsp, 0x55, 0x180D),
    W(Dsp, 0x55, 0x100D),
    W(Dsp, 0x55, 0x180E),
    W(Dsp, 0x55, 0x100E),
    W(Dsp, 0x55, 0x180F),
    W(Dsp, 0x55, 0x100F),
    W(Dsp, 0x55, 0x1810),
    W(Dsp, 0x55, 0x1010),
    W(Dsp, 0x55, 0x1811),
    W(Dsp, 0x55, 0x1011),
    W(Dsp, 0x55, 0x1000),
    // Signal quality measurement.
    W(Dsp, 0x04, 0x16D6),
    W(Dsp, 0x2E, 0x9572),
    W(Dsp, 0x4A, 0x0001),
    W(Dsp, 0x0D, 0x0301),
    R(Dsp, 0xC3),
    // Flag LPS and WUR as idle errors.
    W(Smi, 0x10, 0x0014),
    // Release force-ED, then reset the PCS to run the hardware init.
    W(Smi, 0x1A, 0x0200),
    W(Smi, 0x10, 0x0094),
    Poll(0x10, 0x0080),
    W(Afe, 0x0B, 0x000C),
    R(Smi, 0x18),
    R(Misc, 0x08),
    W(Smi, 0x1A, 0x0300),
];

impl<SPI: SpiDevice, D: DelayNs> Lan937x<SPI, D> {
    /// Read an extended register of `port`'s PHY.
    pub async fn t1_read(&mut self, port: Port, bank: Bank, reg: u8) -> Result<u16, Error<SPI::Error>> {
        if bank == Bank::Smi {
            return self.phy_read(port, reg).await;
        }
        let ctl = EXT_REG_RD | ((bank as u16) << 8) | reg as u16;
        self.t1_dsp_workaround(port, bank).await?;
        self.phy_write(port, EXT_REG_CTL, ctl).await?;
        self.phy_read(port, EXT_REG_RD_DATA).await
    }

    /// Write an extended register of `port`'s PHY.
    pub async fn t1_write(&mut self, port: Port, bank: Bank, reg: u8, value: u16) -> Result<(), Error<SPI::Error>> {
        if bank == Bank::Smi {
            return self.phy_write(port, reg, value).await;
        }
        self.phy_write(port, EXT_REG_WR_DATA, value).await?;
        let ctl = EXT_REG_WR | ((bank as u16) << 8) | reg as u16;
        self.t1_dsp_workaround(port, bank).await?;
        self.phy_write(port, EXT_REG_CTL, ctl).await
    }

    /// Switching into the DSP bank takes two accesses: do a dummy read of DSP
    /// register 0 first (Linux `lan937x_dsp_workaround`).
    async fn t1_dsp_workaround(&mut self, port: Port, bank: Bank) -> Result<(), Error<SPI::Error>> {
        if bank != Bank::Dsp {
            return Ok(());
        }
        let previous = (self.phy_read(port, EXT_REG_CTL).await? >> 8) & 0x7;
        if previous != Bank::Dsp as u16 {
            self.phy_write(port, EXT_REG_CTL, EXT_REG_RD | ((Bank::Dsp as u16) << 8))
                .await?;
        }
        Ok(())
    }

    /// The PHY ID of `port`'s PHY.
    pub async fn t1_phy_id(&mut self, port: Port) -> Result<u32, Error<SPI::Error>> {
        let id1 = self.phy_read(port, PHYID1).await?;
        let id2 = self.phy_read(port, PHYID2).await?;
        Ok(((id1 as u32) << 16) | id2 as u32)
    }

    /// Whether `port`'s link is up.
    pub async fn t1_link(&mut self, port: Port) -> Result<bool, Error<SPI::Error>> {
        Ok(self.phy_read(port, MODE_STS).await? & MODE_STS_LINK != 0)
    }

    /// The configured role of `port`'s PHY.
    pub async fn t1_role(&mut self, port: Port) -> Result<Role, Error<SPI::Error>> {
        Ok(if self.phy_read(port, MS_CTL).await? & MS_CTL_MASTER != 0 {
            Role::Master
        } else {
            Role::Slave
        })
    }

    /// Put `port`'s PHY in near-end loopback, or take it out: frames the switch
    /// sends out of the port come back in on it.
    pub async fn t1_set_loopback(&mut self, port: Port, on: bool) -> Result<(), Error<SPI::Error>> {
        let bmcr = self.phy_read(port, BMCR).await?;
        let bmcr = if on {
            bmcr | BMCR_LOOPBACK
        } else {
            bmcr & !BMCR_LOOPBACK
        };
        self.phy_write(port, BMCR, bmcr).await
    }

    /// The current signal quality of `port`'s link, 0 (worst) to 7 (best).
    pub async fn t1_sqi(&mut self, port: Port) -> Result<u8, Error<SPI::Error>> {
        Ok(((self.t1_read(port, Bank::Dsp, DSP_DCQ_SQI).await? >> 1) & 0x7) as u8)
    }

    /// Acknowledge `port`'s PHY interrupts. Returns the interrupt source bits.
    pub async fn t1_ack_interrupts(&mut self, port: Port) -> Result<u16, Error<SPI::Error>> {
        let _ = self.t1_read(port, Bank::Misc, MISC_INT2_SOURCE).await?;
        self.phy_read(port, INT_SOURCE).await
    }

    /// Enable `port`'s PHY link interrupts (Linux `lan87xx_phy_config_intr`):
    /// link down from the main interrupt source, link up as "communication ready".
    pub async fn t1_enable_link_interrupts(&mut self, port: Port) -> Result<(), Error<SPI::Error>> {
        self.phy_write(port, INT_MASK, 0).await?;
        self.phy_read(port, INT_SOURCE).await?;
        self.t1_write(port, Bank::Misc, MISC_INT2_MASK, 0).await?;
        self.t1_read(port, Bank::Misc, MISC_INT2_SOURCE).await?;
        self.phy_write(port, INT_MASK, INT_LINK_DOWN).await?;
        self.t1_write(port, Bank::Misc, MISC_INT2_MASK, INT2_COMM_READY).await
    }

    /// Run one step of `init` on `port`. Returns `true` when it's done.
    pub async fn t1_init_step(&mut self, port: Port, init: &mut T1Init) -> Result<bool, Error<SPI::Error>> {
        match init.next() {
            None => Ok(true),
            Some(Step::Reset) => {
                self.phy_write(port, BMCR, BMCR_RESET).await?;
                for _ in 0..600 {
                    if self.phy_read(port, BMCR).await? & BMCR_RESET == 0 {
                        self.delay_us(1000).await;
                        return Ok(false);
                    }
                    self.delay_us(1000).await;
                }
                Err(Error::Timeout)
            }
            Some(Step::Role(role)) => {
                let master = if role == Role::Master { MS_CTL_MASTER } else { 0 };
                self.phy_write(port, MS_CTL, MS_CTL_MANUAL | master).await?;
                Ok(false)
            }
            Some(Step::Op(W(bank, reg, value))) => {
                self.t1_write(port, bank, reg, value).await?;
                Ok(false)
            }
            Some(Step::Op(R(bank, reg))) => {
                self.t1_read(port, bank, reg).await?;
                Ok(false)
            }
            Some(Step::Op(Poll(reg, mask))) => {
                for _ in 0..200 {
                    if self.phy_read(port, reg).await? & mask == 0 {
                        return Ok(false);
                    }
                    self.delay_us(150).await;
                }
                Err(Error::Timeout)
            }
            Some(Step::Interrupts) => {
                self.t1_enable_link_interrupts(port).await?;
                Ok(false)
            }
        }
    }

    /// Initialize `port`'s PHY in one go. See [`T1Init`].
    pub async fn t1_init(&mut self, port: Port, role: Role) -> Result<(), Error<SPI::Error>> {
        let mut init = T1Init::new(role);
        while !self.t1_init_step(port, &mut init).await? {}
        Ok(())
    }
}

#[derive(Debug, Clone, Copy)]
enum Step {
    Reset,
    Role(Role),
    Op(Op),
    Interrupts,
}

/// The initialization of a T1 PHY, as steps of a few register accesses each,
/// so that other switch work can run in between.
///
/// Soft reset, master/slave role, Linux's `hw_init`, `slave_init` (followers
/// only) and `phy_init` sequences, then link interrupts. Changing the role
/// takes a full re-initialization, as in Linux; the link drops and retrains.
#[derive(Debug, Clone)]
pub struct T1Init {
    role: Role,
    step: usize,
}

impl T1Init {
    /// Initialize as `role`.
    pub const fn new(role: Role) -> Self {
        Self { role, step: 0 }
    }

    /// The number of steps started so far.
    pub fn steps(&self) -> usize {
        self.step
    }

    fn next(&mut self) -> Option<Step> {
        let slave = if self.role == Role::Slave { SLAVE_INIT } else { &[] };
        let mut i = self.step;
        self.step += 1;
        if i == 0 {
            return Some(Step::Reset);
        }
        i -= 1;
        if i == 0 {
            return Some(Step::Role(self.role));
        }
        i -= 1;
        for table in [HW_INIT, slave, PHY_INIT] {
            if i < table.len() {
                return Some(Step::Op(table[i]));
            }
            i -= table.len();
        }
        if i == 0 {
            return Some(Step::Interrupts);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn init_steps() {
        let mut init = T1Init::new(Role::Master);
        let steps: Vec<_> = core::iter::from_fn(|| init.next()).collect();
        assert_eq!(steps.len(), 2 + HW_INIT.len() + PHY_INIT.len() + 1);
        assert!(matches!(steps[0], Step::Reset));
        assert!(matches!(steps[1], Step::Role(Role::Master)));
        assert!(matches!(steps[2], Step::Op(W(Afe, 0x0B, 0x002D))));
        assert!(matches!(steps.last(), Some(Step::Interrupts)));

        let mut init = T1Init::new(Role::Slave);
        assert_eq!(
            core::iter::from_fn(|| init.next()).count(),
            2 + HW_INIT.len() + SLAVE_INIT.len() + PHY_INIT.len() + 1
        );
        assert_eq!(PHY_INIT.len(), 56);
    }
}
