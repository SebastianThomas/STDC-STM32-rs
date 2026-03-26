use stm32l4xx_hal::hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
};

use super::spi_utils::SpiError;

pub const PAGE_SIZE: u32 = 256;

const READ_DATA_INSTRUCTION: u8 = 0x03;
const PAGE_PROGRAM_INSTRUCTION: u8 = 0x02;
const WRITE_ENABLE_INSTRUCTION: u8 = 0x04;
const ERASE_CHIP_INSTRUCTION: u8 = 0xC7;
const ERASE_64K_INSTRUCTION: u8 = 0xD8;
const ERASE_32K_INSTRUCTION: u8 = 0x52;
const ERASE_4K_INSTRUCTION: u8 = 0x20;

pub trait Flash {
    type Error: core::error::Error;

    fn set_pos(&mut self, new_pos: u32) -> Result<u32, Self::Error>;

    fn write<const BYTES: usize>(&mut self, bytes: &[u8; BYTES]) -> Result<u32, Self::Error>
    where
        [(); 4 + BYTES]: Sized,
        [(); 4 + BYTES + 0]: Sized,
        [(); BYTES + 0]: Sized;

    fn read<const BYTES: usize>(&mut self, position: u32) -> Result<[u8; BYTES], Self::Error>
    where
        [(); 4 + BYTES]: Sized,
        [(); BYTES + 0]: Sized;
}

fn get_24bit_addr(addr: u32) -> [u8; 3] {
    return [
        ((addr >> 16) & 0xFF) as u8,
        ((addr >> 8) & 0xFF) as u8,
        (addr & 0xFF) as u8,
    ];
}

pub struct SpiFlash<SPI: Transfer<u8> + Write<u8>, CSPin: OutputPin, L: Fn(&[u8]) -> ()> {
    current_pos: u32,
    max_addr: u32,
    spi: SPI,
    chip_select_pin: CSPin,

    logger: L,
}

impl<SPI: Transfer<u8> + Write<u8>, CSPin: OutputPin, L: Fn(&[u8]) -> ()> SpiFlash<SPI, CSPin, L> {
    pub fn new(
        current_pos: u32,
        max_addr: u32,
        spi: SPI,
        chip_select_pin: CSPin,
        logger: L,
    ) -> SpiFlash<SPI, CSPin, L> {
        SpiFlash {
            current_pos,
            max_addr,
            spi,
            chip_select_pin,
            logger,
        }
    }

    pub fn write_bytes<const BYTES: usize>(
        &mut self,
        addr: u32,
        bytes: &[u8; BYTES],
    ) -> Result<(), SpiError>
    where
        [(); 4 + BYTES]:,
        [(); BYTES + 0]:,
        [(); 4 + BYTES + 0]:,
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
        let res = self.program_page(addr, bytes);
        let write_res = self.write_enable(false);
        if let Err(e) = res { Err(e) } else { write_res }
    }

    pub fn read_bytes<const BYTES: usize>(&mut self, addr: u32) -> Result<[u8; BYTES], SpiError>
    where
        [(); 4 + BYTES]:,
    {
        let mut buf = [0u8; 4];
        buf[0] = READ_DATA_INSTRUCTION;
        buf[1..=3].copy_from_slice(&get_24bit_addr(addr));
        self.spi_io_operation::<4, { BYTES }>(&buf)
    }

    pub fn erase_chip(&mut self) -> Result<(), SpiError> {
        self.spi_write_operation(&[ERASE_CHIP_INSTRUCTION])
    }

    pub fn erase_64k(&mut self, addr: u32) -> Result<(), SpiError> {
        let mut buf = [0u8; 4];
        buf[0] = ERASE_64K_INSTRUCTION;
        buf[1..=3].copy_from_slice(&get_24bit_addr(addr));
        self.spi_write_operation(&buf)
    }

    pub fn erase_32k(&mut self, addr: u32) -> Result<(), SpiError> {
        let mut buf = [0u8; 4];
        buf[0] = ERASE_32K_INSTRUCTION;
        buf[1..=3].copy_from_slice(&get_24bit_addr(addr));
        self.spi_write_operation(&buf)
    }

    pub fn erase_4k(&mut self, addr: u32) -> Result<(), SpiError> {
        let mut buf = [0u8; 4];
        buf[0] = ERASE_4K_INSTRUCTION;
        buf[1..=3].copy_from_slice(&get_24bit_addr(addr));
        self.spi_write_operation(&buf)
    }

    fn write_enable(&mut self, enable: bool) -> Result<(), SpiError> {
        self.spi_write_operation(&[WRITE_ENABLE_INSTRUCTION | (enable as u8) << 1])
    }

    fn program_page<const BYTES: usize>(
        &mut self,
        addr: u32,
        bytes: &[u8; BYTES],
    ) -> Result<(), SpiError>
    where
        [(); 4 + BYTES]: Sized,
        [(); 4 + BYTES + 0]: Sized,
        [(); BYTES + 0]: Sized,
    {
        let mut buf: [u8; 4 + BYTES] = [0; 4 + BYTES];
        let addr_buf = get_24bit_addr(addr);
        buf[0] = PAGE_PROGRAM_INSTRUCTION;
        buf[1..=3].copy_from_slice(&addr_buf);
        buf[4..=3 + BYTES].copy_from_slice(bytes);
        self.spi_write_operation::<{ 4 + BYTES }>(&buf)
    }

    /**
     * Operations without Data In
     */
    fn spi_write_operation<const BYTES: usize>(
        &mut self,
        op_data: &[u8; BYTES],
    ) -> Result<(), SpiError>
    where
        [(); BYTES + 0]:,
    {
        self.spi_io_operation::<BYTES, 0>(op_data).map(|_| {})
    }

    /**
     * Operations with Data In
     */
    fn spi_io_operation<const WBYTES: usize, const RBYTES: usize>(
        &mut self,
        op_data: &[u8; WBYTES],
    ) -> Result<[u8; RBYTES], SpiError>
    where
        [(); WBYTES + RBYTES]:,
    {
        // CS Low
        let low = self.chip_select_pin.set_low();
        if let Err(_) = low {
            let msg = "Failed setting CS to low";
            let _ = (self.logger)(msg.as_bytes());
            return Err(SpiError::new(1, msg));
        };

        let res = if RBYTES == 0 {
            match self.spi.write(op_data) {
                Ok(_) => Ok([0; RBYTES]),
                Err(_) => {
                    let msg = "Failed executing write operation";
                    let _ = (self.logger)(msg.as_bytes());
                    Err(SpiError::new(0, msg))
                }
            }
        } else {
            let mut buf = [0u8; WBYTES + RBYTES];

            // Fill command + address
            buf[0..=WBYTES].copy_from_slice(op_data);
            // Remaining bytes = dummy writes (0x00)
            for i in 0..RBYTES {
                buf[WBYTES + i] = 0x00;
            }
            match self.spi.transfer(&mut buf) {
                Ok(_) => {
                    let mut res = [0u8; RBYTES];
                    res.copy_from_slice(&buf[WBYTES..]);
                    Ok(res)
                }
                Err(_) => {
                    let msg = "Failed executing rw operation";
                    let _ = (self.logger)(msg.as_bytes());
                    Err(SpiError::new(0, msg))
                }
            }
        };

        // CS High
        let high = self.chip_select_pin.set_high();
        let _high_res: Result<_, SpiError> = match high {
            Ok(v) => Ok(v),
            Err(_) => {
                let msg = "Failed setting CS to high";
                let _ = (self.logger)(msg.as_bytes());
                return Err(SpiError::new(3, msg));
            }
        };

        res
    }
}

impl<SPI: Transfer<u8> + Write<u8>, CSPin: OutputPin, L: Fn(&[u8]) -> ()> Flash
    for SpiFlash<SPI, CSPin, L>
{
    type Error = SpiError;

    fn set_pos(&mut self, new_pos: u32) -> Result<u32, Self::Error> {
        if self.max_addr > new_pos {
            return Err(SpiError {
                priority: 0,
                details: "New address out of range for given Flash chip.",
            });
        }
        self.max_addr = new_pos;
        Ok(self.max_addr)
    }

    fn write<const BYTES: usize>(&mut self, data: &[u8; BYTES]) -> Result<u32, Self::Error>
    where
        [(); 4 + BYTES]: Sized,
        [(); 4 + BYTES + 0]: Sized,
        [(); BYTES + 0]: Sized,
    {
        let bytes = BYTES as u32;
        let start_addr = self.current_pos;
        if self.current_pos + bytes > self.max_addr {
            let msg = "Out of Bounds write attempted, aborting";
            let _ = (self.logger)(msg.as_bytes());
            return Err(SpiError::new(0, msg));
        }
        self.write_bytes(start_addr, data)?;
        self.current_pos += bytes;
        Ok(start_addr)
    }

    fn read<const BYTES: usize>(&mut self, pos: u32) -> Result<[u8; BYTES], Self::Error>
    where
        [(); 4 + BYTES]: Sized,
        [(); BYTES + 0]: Sized,
    {
        let bytes = BYTES as u32;
        if pos + bytes > self.max_addr {
            let msg = "Out of Bounds read attempted, aborting";
            let _ = (self.logger)(msg.as_bytes());
            return Err(SpiError::new(0, msg));
        }
        self.read_bytes(pos)
    }
}
