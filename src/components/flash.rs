use rtt_target::rprintln;
use stm32l4xx_hal::hal::{
    blocking::spi::{Transfer, Write},
    digital::v2::OutputPin,
};

use crate::components::spi_utils::DetailsError;

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
    type Error: core::error::Error + DetailsError;

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

pub struct SpiFlash<SPI: Transfer<u8> + Write<u8>, CSPin: OutputPin> {
    current_pos: u32,
    max_addr: u32,
    spi: SPI,
    chip_select_pin: CSPin,
}

impl<SPI: Transfer<u8> + Write<u8>, CSPin: OutputPin> SpiFlash<SPI, CSPin> {
    pub fn new(
        current_pos: u32,
        max_addr: u32,
        spi: SPI,
        chip_select_pin: CSPin,
    ) -> SpiFlash<SPI, CSPin> {
        SpiFlash {
            current_pos,
            max_addr,
            spi,
            chip_select_pin,
        }
    }

    /// Delegate to the slice-based multi-page writer. This allows writes
    /// that cross page boundaries. Note: multi-page writes are NOT atomic
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
        self.write_bytes_slice(addr, bytes.as_slice())
    }

    pub fn read_bytes<const BYTES: usize>(&mut self, addr: u32) -> Result<[u8; BYTES], SpiError>
    where
        [(); 4 + BYTES]:,
    {
        let mut out = [0u8; BYTES];
        // Use chunked reader to support arbitrarily large reads.
        self.read_bytes_slice(addr, &mut out)?;
        Ok(out)
    }

    pub fn erase_chip(&mut self) -> Result<(), SpiError> {
        self.erase_with_instruction(ERASE_CHIP_INSTRUCTION, 0)
    }

    pub fn erase_64k(&mut self, addr: u32) -> Result<(), SpiError> {
        self.erase_with_instruction(ERASE_64K_INSTRUCTION, addr)
    }

    pub fn erase_32k(&mut self, addr: u32) -> Result<(), SpiError> {
        self.erase_with_instruction(ERASE_32K_INSTRUCTION, addr)
    }

    pub fn erase_4k(&mut self, addr: u32) -> Result<(), SpiError> {
        self.erase_with_instruction(ERASE_4K_INSTRUCTION, addr)
    }

    fn erase_with_instruction(&mut self, instruction: u8, addr: u32) -> Result<(), SpiError> {
        let mut buf = [0u8; 4];
        buf[0] = instruction;
        buf[1..=3].copy_from_slice(&get_24bit_addr(addr));
        self.write_enable(true)?;
        let erase_res = self.spi_write_operation(&buf);
        let disable_res = self.write_enable(false);
        match erase_res {
            Err(e) => Err(e),
            Ok(_) => disable_res,
        }
    }

    fn write_enable(&mut self, enable: bool) -> Result<(), SpiError> {
        self.spi_write_operation(&[WRITE_ENABLE_INSTRUCTION | (enable as u8) << 1])
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
            rprintln!("{}", msg);
            return Err(SpiError::new(1, msg));
        };

        let res = if RBYTES == 0 {
            match self.spi.write(op_data) {
                Ok(_) => Ok([0; RBYTES]),
                Err(_) => {
                    let msg = "Failed executing write operation";
                    rprintln!("{}", msg);
                    Err(SpiError::new(0, msg))
                }
            }
        } else {
            let mut buf = [0u8; WBYTES + RBYTES];

            // Fill command + address
            buf[0..WBYTES].copy_from_slice(op_data);
            rprintln!("Copied from slice");
            // Remaining bytes = dummy writes (0x00)
            for i in 0..RBYTES {
                buf[WBYTES + i] = 0x00;
            }
            rprintln!("Start transfer");
            match self.spi.transfer(&mut buf) {
                Ok(_) => {
                    let mut res = [0u8; RBYTES];
                    res.copy_from_slice(&buf[WBYTES..]);
                    Ok(res)
                }
                Err(_) => {
                    let msg = "Failed executing rw operation";
                    rprintln!("{}", msg);
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
                rprintln!("{}", msg);
                return Err(SpiError::new(3, msg));
            }
        };

        res
    }
}

impl<SPI: Transfer<u8> + Write<u8>, CSPin: OutputPin> SpiFlash<SPI, CSPin> {
    /// Write an arbitrary-length buffer to the flash, splitting across page
    /// boundaries as necessary. This is not atomic: if an error occurs part
    /// way through, earlier pages may already be programmed.
    pub fn write_bytes_slice(&mut self, mut addr: u32, mut data: &[u8]) -> Result<(), SpiError> {
        while !data.is_empty() {
            let offset = (addr % PAGE_SIZE) as usize;
            let space = (PAGE_SIZE as usize) - offset;
            let write_len = data.len().min(space);

            // Enable writes for this page, send program command + address, then write data.
            self.write_enable(true)?;

            // CS Low
            if let Err(_) = self.chip_select_pin.set_low() {
                let msg = "Failed setting CS to low";
                rprintln!("{}", msg);
                return Err(SpiError::new(1, msg));
            }

            let addr_buf = get_24bit_addr(addr);
            let header = [
                PAGE_PROGRAM_INSTRUCTION,
                addr_buf[0],
                addr_buf[1],
                addr_buf[2],
            ];
            if let Err(_) = self.spi.write(&header) {
                let msg = "Failed executing write operation (header)";
                rprintln!("{}", msg);
                let _ = self.chip_select_pin.set_high();
                return Err(SpiError::new(0, msg));
            }

            if let Err(_) = self.spi.write(&data[..write_len]) {
                let msg = "Failed executing write operation (data)";
                rprintln!("{}", msg);
                let _ = self.chip_select_pin.set_high();
                return Err(SpiError::new(0, msg));
            }

            // CS High
            if let Err(_) = self.chip_select_pin.set_high() {
                let msg = "Failed setting CS to high";
                rprintln!("{}", msg);
                return Err(SpiError::new(3, msg));
            }

            // Disable write (mirror previous behavior)
            self.write_enable(false)?;

            addr = addr.wrapping_add(write_len as u32);
            data = &data[write_len..];
        }

        Ok(())
    }

    /// Read arbitrary-length data from flash into `out`, in page-sized chunks.
    pub fn read_bytes_slice(&mut self, mut addr: u32, out: &mut [u8]) -> Result<(), SpiError> {
        // Chunk reads to page-size to avoid large stack buffers
        const CHUNK_MAX: usize = 256; // PAGE_SIZE
        let mut dest = 0usize;
        while dest < out.len() {
            let remaining = out.len() - dest;
            let chunk = remaining.min(CHUNK_MAX);

            // Build transfer buffer: 1-byte cmd + 3-byte addr + chunk dummy bytes
            let mut buf = [0u8; 4 + CHUNK_MAX];
            buf[0] = READ_DATA_INSTRUCTION;
            let addr_buf = get_24bit_addr(addr);
            buf[1] = addr_buf[0];
            buf[2] = addr_buf[1];
            buf[3] = addr_buf[2];
            for i in 0..chunk {
                buf[4 + i] = 0x00;
            }

            // CS Low
            if let Err(_) = self.chip_select_pin.set_low() {
                let msg = "Failed setting CS to low";
                rprintln!("{}", msg);
                return Err(SpiError::new(1, msg));
            }

            match self.spi.transfer(&mut buf[..4 + chunk]) {
                Ok(_) => {
                    out[dest..dest + chunk].copy_from_slice(&buf[4..4 + chunk]);
                }
                Err(_) => {
                    let msg = "Failed executing rw operation";
                    rprintln!("{}", msg);
                    let _ = self.chip_select_pin.set_high();
                    return Err(SpiError::new(0, msg));
                }
            }

            // CS High
            if let Err(_) = self.chip_select_pin.set_high() {
                let msg = "Failed setting CS to high";
                rprintln!("{}", msg);
                return Err(SpiError::new(3, msg));
            }

            addr = addr.wrapping_add(chunk as u32);
            dest += chunk;
        }

        Ok(())
    }
}

impl<SPI: Transfer<u8> + Write<u8>, CSPin: OutputPin> Flash for SpiFlash<SPI, CSPin> {
    type Error = SpiError;

    fn set_pos(&mut self, new_pos: u32) -> Result<u32, Self::Error> {
        if self.max_addr > new_pos {
            return Err(SpiError {
                priority: 0,
                details: "New address out of range for given Flash chip.",
            });
        }
        self.current_pos = new_pos;
        Ok(self.current_pos)
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
            rprintln!("{}", msg);
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
            rprintln!("{}", msg);
            return Err(SpiError::new(0, msg));
        }
        self.read_bytes(pos)
    }
}
