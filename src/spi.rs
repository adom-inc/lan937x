//! SPI register access.
//!
//! Every transaction starts with a 32-bit command: a 3-bit opcode, a 24-bit
//! register address and 5 turnaround bits, most significant bit first. Data
//! follows, big-endian, and the address auto-increments for longer accesses.
//! SPI mode 0, at most 50 MHz (10 MHz while accessing PHY registers through
//! the indirect window).

use embedded_hal_async::spi::{Operation, SpiDevice};

use crate::Error;

const OP_WRITE: u32 = 0b010;
const OP_READ: u32 = 0b011;
const ADDR_SHIFT: u32 = 5;
const OP_SHIFT: u32 = 29;

fn command(op: u32, addr: u32) -> [u8; 4] {
    ((op << OP_SHIFT) | ((addr & 0x00ff_ffff) << ADDR_SHIFT)).to_be_bytes()
}

/// Register access to a LAN937x over SPI.
pub struct Spi<SPI> {
    spi: SPI,
}

impl<SPI: SpiDevice> Spi<SPI> {
    /// Use `spi`, which must be configured for mode 0.
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Give the SPI device back.
    pub fn release(self) -> SPI {
        self.spi
    }

    /// Read `buf.len()` consecutive bytes starting at `addr`.
    pub async fn read_bytes(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), Error<SPI::Error>> {
        let cmd = command(OP_READ, addr);
        self.spi
            .transaction(&mut [Operation::Write(&cmd), Operation::Read(buf)])
            .await
            .map_err(Error::Spi)
    }

    /// Write `data` to consecutive bytes starting at `addr`.
    pub async fn write_bytes(&mut self, addr: u32, data: &[u8]) -> Result<(), Error<SPI::Error>> {
        let cmd = command(OP_WRITE, addr);
        self.spi
            .transaction(&mut [Operation::Write(&cmd), Operation::Write(data)])
            .await
            .map_err(Error::Spi)
    }

    /// Read an 8-bit register.
    pub async fn read8(&mut self, addr: u32) -> Result<u8, Error<SPI::Error>> {
        let mut buf = [0; 1];
        self.read_bytes(addr, &mut buf).await?;
        Ok(buf[0])
    }

    /// Read a 16-bit register.
    pub async fn read16(&mut self, addr: u32) -> Result<u16, Error<SPI::Error>> {
        let mut buf = [0; 2];
        self.read_bytes(addr, &mut buf).await?;
        Ok(u16::from_be_bytes(buf))
    }

    /// Read a 32-bit register.
    pub async fn read32(&mut self, addr: u32) -> Result<u32, Error<SPI::Error>> {
        let mut buf = [0; 4];
        self.read_bytes(addr, &mut buf).await?;
        Ok(u32::from_be_bytes(buf))
    }

    /// Write an 8-bit register.
    pub async fn write8(&mut self, addr: u32, value: u8) -> Result<(), Error<SPI::Error>> {
        self.write_bytes(addr, &[value]).await
    }

    /// Write a 16-bit register.
    pub async fn write16(&mut self, addr: u32, value: u16) -> Result<(), Error<SPI::Error>> {
        self.write_bytes(addr, &value.to_be_bytes()).await
    }

    /// Write a 32-bit register.
    pub async fn write32(&mut self, addr: u32, value: u32) -> Result<(), Error<SPI::Error>> {
        self.write_bytes(addr, &value.to_be_bytes()).await
    }

    /// Clear the bits in `clear` and set the bits in `set` of an 8-bit register.
    pub async fn modify8(&mut self, addr: u32, clear: u8, set: u8) -> Result<u8, Error<SPI::Error>> {
        let old = self.read8(addr).await?;
        self.write8(addr, (old & !clear) | set).await?;
        Ok(old)
    }

    /// Clear the bits in `clear` and set the bits in `set` of a 16-bit register.
    pub async fn modify16(&mut self, addr: u32, clear: u16, set: u16) -> Result<u16, Error<SPI::Error>> {
        let old = self.read16(addr).await?;
        self.write16(addr, (old & !clear) | set).await?;
        Ok(old)
    }

    /// Clear the bits in `clear` and set the bits in `set` of a 32-bit register.
    pub async fn modify32(&mut self, addr: u32, clear: u32, set: u32) -> Result<u32, Error<SPI::Error>> {
        let old = self.read32(addr).await?;
        self.write32(addr, (old & !clear) | set).await?;
        Ok(old)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_words() {
        // Linux ksz_spi.c: (op << 29) | (addr << 5).
        assert_eq!(command(OP_READ, 0x0000), [0x60, 0, 0, 0]);
        assert_eq!(command(OP_WRITE, 0x0000), [0x40, 0, 0, 0]);
        assert_eq!(command(OP_READ, 0x0210), [0x60, 0x00, 0x42, 0x00]);
        assert_eq!(command(OP_WRITE, 0x6301), [0x40, 0x0c, 0x60, 0x20]);
    }
}
