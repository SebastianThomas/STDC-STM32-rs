use core::fmt::{Display, Formatter};

use stm32l4xx_hal::hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
};

use crate::components::uart_log::ExternalLogger;

pub const PAGE_SIZE: u32 = 256;

const PAGE_PROGRAM_INSTRUCTION: u8 = 0x02;
const WRITE_ENABLE_INSTRUCTION: u8 = 0x04;
const ERASE_CHIP_INSTRUCTION: u8 = 0xC7;
const ERASE_64K_INSTRUCTION: u8 = 0xD8;
const ERASE_32K_INSTRUCTION: u8 = 0x52;
const ERASE_4K_INSTRUCTION: u8 = 0x20;

pub trait Flash {
    type Error: core::error::Error;

    fn write<const BYTES: usize>(&mut self, bytes: &[u8; BYTES]) -> Result<u32, Self::Error>
    where
        [(); 3 + BYTES]:;
}

#[derive(Debug)]
pub struct SpiError {
    pub priority: u8,
    pub details: &'static str,
}

impl SpiError {
    fn new(priority: u8, details: &'static str) -> SpiError {
        SpiError { priority, details }
    }
}

impl Display for SpiError {
    fn fmt(&self, f: &mut Formatter) -> core::fmt::Result {
        write!(f, "SPI Flash Error: {}", self.details)
    }
}

impl core::error::Error for SpiError {}

fn get_24bit_addr(addr: u32) -> [u8; 3] {
    return [
        ((addr >> 16) & 0xFF) as u8,
        ((addr >> 8) & 0xFF) as u8,
        (addr & 0xFF) as u8,
    ];
}

pub struct SpiFlash<'l, SPI: Transfer<u8> + Write<u8>, CSPin: OutputPin, L: ExternalLogger> {
    current_pos: u32,
    max_addr: u32,
    spi: SPI,
    chip_select_pin: CSPin,

    logger: &'l mut L,
}

impl<SPI: Transfer<u8> + Write<u8>, CSPin: OutputPin, L: ExternalLogger>
    SpiFlash<'_, SPI, CSPin, L>
{
    fn write_bytes<const BYTES: usize>(
        &mut self,
        addr: u32,
        bytes: &[u8; BYTES],
    ) -> Result<(), SpiError>
    where
        [(); 3 + BYTES]:,
    {
        let offset = addr % PAGE_SIZE;
        // let base = addr - offset;
        if PAGE_SIZE - offset < BYTES as u32 {
            return Err(SpiError::new(
                0,
                "Not enough space on current page to write bytes.",
            ));
        }
        self.write_enable(true)?;
        self.program_page(addr, bytes)?;
        self.write_enable(false)?;
        Ok(())
    }

    pub fn erase_chip(&mut self) -> Result<(), SpiError> {
        self.spi_write_operation(ERASE_CHIP_INSTRUCTION, &[])
    }

    pub fn erase_64k(&mut self, addr: u32) -> Result<(), SpiError> {
        self.spi_write_operation(ERASE_64K_INSTRUCTION, &get_24bit_addr(addr))
    }

    pub fn erase_32k(&mut self, addr: u32) -> Result<(), SpiError> {
        self.spi_write_operation(ERASE_32K_INSTRUCTION, &get_24bit_addr(addr))
    }

    pub fn erase_4k(&mut self, addr: u32) -> Result<(), SpiError> {
        self.spi_write_operation(ERASE_4K_INSTRUCTION, &get_24bit_addr(addr))
    }

    fn write_enable(&mut self, enable: bool) -> Result<(), SpiError> {
        self.spi_write_operation(WRITE_ENABLE_INSTRUCTION | (enable as u8) << 1, &[])
    }

    fn program_page<const BYTES: usize>(
        &mut self,
        addr: u32,
        bytes: &[u8; BYTES],
    ) -> Result<(), SpiError>
    where
        [(); 3 + BYTES]:,
    {
        let mut buf: [u8; 3 + BYTES] = [0; 3 + BYTES];
        let addr_buf = get_24bit_addr(addr);
        buf[0..2].copy_from_slice(&addr_buf);
        buf[3..2 + BYTES].copy_from_slice(bytes);
        self.spi_write_operation(PAGE_PROGRAM_INSTRUCTION, &buf)
    }

    /**
     * Operations without Data In
     */
    fn spi_write_operation<const BYTES: usize>(
        &mut self,
        op: u8,
        data: &[u8; BYTES],
    ) -> Result<(), SpiError> {
        // CS Low
        let low = self.chip_select_pin.set_low();
        if let Err(_) = low {
            let msg = "Failed setting CS to low";
            let _ = self.logger.log_bytes(msg.as_bytes());
            return Err(SpiError::new(1, msg));
        };
        let write_res_op = self.spi.write(&[op]);
        let write_res_op: Result<_, SpiError> = match write_res_op {
            Ok(_) => Ok(()),
            Err(_) => {
                let msg = "Failed executing write for op bytes, aborting.";
                let _ = self.logger.log_bytes(msg.as_bytes());
                return Err(SpiError::new(0, msg));
            }
        };
        let write_res = if write_res_op.is_ok() {
            let write_res = self.spi.write(data);
            match write_res {
                Ok(_) => Ok(()),
                Err(_) => {
                    let msg = "Failed executing write operation";
                    let _ = self.logger.log_bytes(msg.as_bytes());
                    Err(SpiError::new(0, msg))
                }
            }
        } else {
            Ok(())
        };

        // CS High
        let high = self.chip_select_pin.set_high();
        let high_res: Result<_, SpiError> = match high {
            Ok(v) => Ok(v),
            Err(_) => {
                let msg = "Failed setting CS to high";
                let _ = self.logger.log_bytes(msg.as_bytes());
                return Err(SpiError::new(3, msg));
            }
        };

        match (write_res_op, write_res, high_res) {
            (Ok(_), Ok(_), Ok(_)) => Ok(()),
            (Err(e), _, _) => Err(e),
            (_, Err(e), _) => Err(e),
            (_, _, e) => e,
        }
    }
}

impl<SPI: Transfer<u8> + Write<u8>, CSPin: OutputPin, L: ExternalLogger> Flash
    for SpiFlash<'_, SPI, CSPin, L>
{
    type Error = SpiError;

    fn write<const BYTES: usize>(&mut self, bytes: &[u8; BYTES]) -> Result<u32, SpiError>
    where
        [(); 3 + BYTES]:,
    {
        let start_addr = self.current_pos;
        if self.current_pos + BYTES as u32 > self.max_addr {
            let msg = "Out of Bounds write attempted, aborting";
            let _ = self.logger.log_bytes(msg.as_bytes());
            return Err(SpiError::new(0, msg));
        }
        self.write_bytes(start_addr, bytes)?;
        self.current_pos += BYTES as u32;
        Ok(start_addr)
    }
}
