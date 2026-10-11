//! The PTP hardware clock and timestamps.
//!
//! The clock adds 40 ns per 25 MHz tick, plus a 32-bit sub-nanosecond rate
//! accumulator. Timestamps in tail tags and in the per-port transmit
//! timestamp registers are partial: bits 31:30 are the seconds modulo 4, bits
//! 29:0 the nanoseconds. A full time is rebuilt from a recent reading of the
//! clock ([`reconstruct`]).

use embedded_hal_async::delay::DelayNs;
use embedded_hal_async::spi::SpiDevice;

use crate::{Error, Lan937x, Port, reg};

const NANOS_PER_SEC: u32 = 1_000_000_000;

/// A time of the switch's clock.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub struct PhcTime {
    /// Seconds. The clock's seconds are 32 bits wide.
    pub sec: u32,
    /// Nanoseconds, below 1e9.
    pub ns: u32,
}

impl PhcTime {
    /// Nanoseconds since the clock's epoch.
    pub const fn as_nanos(self) -> i64 {
        self.sec as i64 * NANOS_PER_SEC as i64 + self.ns as i64
    }
}

/// Rebuild a full time from a partial timestamp, given a reading of the clock
/// within about two seconds of it (Linux `ksz_tstamp_reconstruct`).
pub fn reconstruct(raw: u32, reference: PhcTime) -> PhcTime {
    let ns = raw & 0x3fff_ffff;
    let mut sec = (reference.sec & !3) | (raw >> 30);
    let delta = (sec as i64 - reference.sec as i64) * NANOS_PER_SEC as i64 + ns as i64 - reference.ns as i64;
    if delta > 2 * NANOS_PER_SEC as i64 {
        sec = sec.wrapping_sub(4);
    } else if delta < -2 * (NANOS_PER_SEC as i64) {
        sec = sec.wrapping_add(4);
    }
    PhcTime { sec, ns }
}

/// The transmit timestamp registers: one per class per port, so at most one
/// timestamp per class can be outstanding on a port.
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TxClass {
    /// Sync.
    Sync,
    /// Delay_Req and Pdelay_Req.
    Xdelay,
    /// Pdelay_Resp.
    PdelayResp,
}

impl TxClass {
    /// All classes.
    pub const ALL: [TxClass; 3] = [TxClass::Sync, TxClass::Xdelay, TxClass::PdelayResp];

    /// The class's bit in `P_PTP_TS_INT_STS`/`EN`.
    pub const fn interrupt(self) -> u16 {
        match self {
            TxClass::Sync => reg::PTP_TS_INT_SYNC,
            TxClass::Xdelay => reg::PTP_TS_INT_XDELAY,
            TxClass::PdelayResp => reg::PTP_TS_INT_PDRESP,
        }
    }

    /// The class's timestamp register.
    pub const fn register(self) -> u16 {
        match self {
            TxClass::Sync => reg::P_PTP_SYNC_TS,
            TxClass::Xdelay => reg::P_PTP_XDELAY_TS,
            TxClass::PdelayResp => reg::P_PTP_PDRESP_TS,
        }
    }

    /// An index, for tables.
    pub const fn index(self) -> usize {
        self as usize
    }
}

/// The transmit timestamp class of a frame: a PTP event message over IEEE
/// 802.3 (ethertype 0x88F7) that the switch timestamps. `None` otherwise.
pub fn classify(frame: &[u8]) -> Option<TxClass> {
    if frame.len() < 15 || frame[12..14] != [0x88, 0xf7] {
        return None;
    }
    match frame[14] & 0x0f {
        0x0 => Some(TxClass::Sync),
        0x1 | 0x2 => Some(TxClass::Xdelay),
        0x3 => Some(TxClass::PdelayResp),
        _ => None,
    }
}

/// The `PTP_SUBNANOSEC_RATE` value for a frequency adjustment in parts per
/// million scaled by 2^16 (as `statime` and Linux give it), or `None` for no
/// adjustment. Saturates at what the 30-bit field holds.
pub fn rate_register(scaled_ppm: i64) -> Option<u32> {
    if scaled_ppm == 0 {
        return None;
    }
    // Per 40 ns tick, in 2^-32 ns: 40 * 2^32 * ppm / 1e6 = 40 * 2^16 * scaled_ppm / 1e6.
    let adj = (40u128 << 16) * scaled_ppm.unsigned_abs() as u128 / 1_000_000;
    let adj = adj.min(0x3fff_ffff) as u32;
    Some(if scaled_ppm > 0 { adj | reg::PTP_RATE_DIR } else { adj })
}

impl<SPI: SpiDevice, D: DelayNs> Lan937x<SPI, D> {
    /// Read the clock.
    pub async fn ptp_read(&mut self) -> Result<PhcTime, Error<SPI::Error>> {
        self.regs().modify16(reg::PTP_CLK_CTL, 0, reg::PTP_CLK_READ).await?;
        let phase = (self.regs().read16(reg::PTP_RTC_PHASE).await? & 0x7) as u32;
        let ns = self.regs().read32(reg::PTP_RTC_NANOSEC).await?;
        let sec = self.regs().read32(reg::PTP_RTC_SEC).await?;
        let ns = ns + 8 * phase;
        Ok(if ns >= NANOS_PER_SEC {
            PhcTime {
                sec: sec.wrapping_add(1),
                ns: ns - NANOS_PER_SEC,
            }
        } else {
            PhcTime { sec, ns }
        })
    }

    /// Set the clock.
    pub async fn ptp_set(&mut self, time: PhcTime) -> Result<(), Error<SPI::Error>> {
        self.regs().write16(reg::PTP_RTC_PHASE, 0).await?;
        self.regs().write32(reg::PTP_RTC_NANOSEC, time.ns).await?;
        self.regs().write32(reg::PTP_RTC_SEC, time.sec).await?;
        self.regs().modify16(reg::PTP_CLK_CTL, 0, reg::PTP_CLK_LOAD).await?;
        Ok(())
    }

    /// Step the clock by `delta` nanoseconds.
    pub async fn ptp_step(&mut self, delta: i64) -> Result<(), Error<SPI::Error>> {
        let magnitude = delta.unsigned_abs();
        let sec = (magnitude / NANOS_PER_SEC as u64) as u32;
        let ns = (magnitude % NANOS_PER_SEC as u64) as u32;
        self.regs().write32(reg::PTP_RTC_NANOSEC, ns).await?;
        self.regs().write32(reg::PTP_RTC_SEC, sec).await?;
        let add = if delta >= 0 { reg::PTP_CLK_STEP_DIR } else { 0 };
        self.regs()
            .modify16(reg::PTP_CLK_CTL, reg::PTP_CLK_STEP_DIR, reg::PTP_CLK_STEP_ADJ | add)
            .await?;
        Ok(())
    }

    /// Adjust the clock's frequency by `scaled_ppm` (ppm × 2^16).
    pub async fn ptp_adjust_rate(&mut self, scaled_ppm: i64) -> Result<(), Error<SPI::Error>> {
        match rate_register(scaled_ppm) {
            Some(rate) => {
                self.regs().write32(reg::PTP_SUBNANOSEC_RATE, rate).await?;
                self.regs()
                    .modify16(reg::PTP_CLK_CTL, 0, reg::PTP_CLK_ADJ_ENABLE)
                    .await?;
            }
            None => {
                self.regs()
                    .modify16(reg::PTP_CLK_CTL, reg::PTP_CLK_ADJ_ENABLE, 0)
                    .await?;
            }
        }
        Ok(())
    }

    /// Enable the transmit timestamp interrupts of `port`, all classes.
    pub async fn ptp_enable_tx_interrupts(&mut self, port: Port) -> Result<(), Error<SPI::Error>> {
        let all = TxClass::ALL.iter().fold(0, |m, c| m | c.interrupt());
        self.regs().write16(reg::port(port, reg::P_PTP_TS_INT_STS), all).await?;
        self.regs().write16(reg::port(port, reg::P_PTP_TS_INT_EN), all).await
    }

    /// Take `port`'s pending transmit timestamps: acknowledge them and return
    /// the raw (partial) timestamp of each class that had one.
    pub async fn ptp_take_tx_timestamps(&mut self, port: Port) -> Result<[Option<u32>; 3], Error<SPI::Error>> {
        let status_reg = reg::port(port, reg::P_PTP_TS_INT_STS);
        let status = self.regs().read16(status_reg).await?;
        let mut out = [None; 3];
        for class in TxClass::ALL {
            if status & class.interrupt() != 0 {
                out[class.index()] = Some(self.regs().read32(reg::port(port, class.register())).await?);
            }
        }
        if status != 0 {
            self.regs().write16(status_reg, status).await?;
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn t(sec: u32, ns: u32) -> PhcTime {
        PhcTime { sec, ns }
    }

    fn raw(sec: u32, ns: u32) -> u32 {
        ((sec & 3) << 30) | ns
    }

    #[test]
    fn reconstruct_near_the_reference() {
        assert_eq!(reconstruct(raw(101, 5), t(101, 7)), t(101, 5));
        assert_eq!(reconstruct(raw(102, 999_999_999), t(101, 500)), t(102, 999_999_999));
        assert_eq!(reconstruct(raw(99, 0), t(100, 10)), t(99, 0));
    }

    #[test]
    fn reconstruct_across_the_modulo_4_boundary() {
        // Reference at 103.9 s (103 & !3 = 100); event at 104.1 s.
        assert_eq!(
            reconstruct(raw(104, 100_000_000), t(103, 900_000_000)),
            t(104, 100_000_000)
        );
        // Reference at 104.1 s; event at 103.9 s.
        assert_eq!(
            reconstruct(raw(103, 900_000_000), t(104, 100_000_000)),
            t(103, 900_000_000)
        );
        // Reference at 0.5 s; event just before, at 0.1 s.
        assert_eq!(reconstruct(raw(0, 100), t(0, 500)), t(0, 100));
    }

    #[test]
    fn classify_ptp_messages() {
        let mut frame = [0u8; 60];
        frame[12..14].copy_from_slice(&[0x88, 0xf7]);
        for (msg, class) in [
            (0x0, Some(TxClass::Sync)),
            (0x1, Some(TxClass::Xdelay)),
            (0x2, Some(TxClass::Xdelay)),
            (0x3, Some(TxClass::PdelayResp)),
            (0x8, None), // Follow_Up
            (0xb, None), // Announce
        ] {
            frame[14] = 0x10 | msg; // majorSdoId in the high nibble
            assert_eq!(classify(&frame), class);
        }
        frame[12] = 0x08;
        assert_eq!(classify(&frame), None);
    }

    #[test]
    fn rate_registers() {
        assert_eq!(rate_register(0), None);
        // 1 ppm: 40 ns * 1e-6 = 40e-6 ns per tick = 171798.69 * 2^-32 ns.
        assert_eq!(rate_register(1 << 16), Some(0x8000_0000 | 171_798));
        assert_eq!(rate_register(-(1 << 16)), Some(171_798));
        assert_eq!(rate_register(i64::MAX), Some(0xbfff_ffff));
    }
}
