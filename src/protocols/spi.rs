use rtt_target::rprintln;
use stm32l4xx_hal::hal::blocking::spi::{Transfer, Write};
use stm32l4xx_hal::hal::digital::v2::OutputPin;

/**
 * Write `cmd` to the `spi`, then read into `buf`. Sets `cs` to low before transmit and high after
 * recv to select/deselect the device.
 */
pub fn spi_read_register<SPI, CS>(spi: &mut SPI, cs: &mut CS, buf: &mut [u8])
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    // select device
    if let Err(_e) = cs.set_low() {
        rprintln!("Failed to select device.");
        panic!("Failed to select device");
    }
    // read/write data into buffer
    if let Err(_e) = spi.transfer(buf) {
        rprintln!("Failed to read.");
        panic!("Failed to read");
    }
    // deselect device
    if let Err(_e) = cs.set_high() {
        rprintln!("Failed to deselect device.");
        panic!("Failed to deselect device");
    }
}

pub fn spi_write_delay<SPI, CS, F>(spi: &mut SPI, cs: &mut CS, cmd: u8, delay_us: F)
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
    F: Fn(),
{
    cs.set_low().ok();
    spi.write(&[cmd]).ok();
    delay_us();
    cs.set_high().ok();
}

/** Attempt to communicate with SPI devices on given CS pins.
 * `cs_pins` can be any iterable of OutputPin implementors.
 * `id_cmd` is the command to read device ID / PROM register.
 */
pub fn scan_spi<SPI, CS>(spi: &mut SPI, cs_pins: &mut [&mut CS], id_cmd: u8)
where
    SPI: Transfer<u8> + Write<u8>,
    CS: OutputPin,
{
    let mut buf = [0u8; 2]; // adjust size for device ID / PROM
    for (i, cs) in cs_pins.iter_mut().enumerate() {
        cs.set_low().ok();
        // send ID command
        if spi.write(&[id_cmd]).is_ok() {
            // try reading data
            if spi.transfer(&mut buf).is_ok() {
                rprintln!("SPI device {} responded: {:02X} {:02X}", i, buf[0], buf[1]);
            } else {
                rprintln!("SPI device {}: failed to read", i);
            }
        } else {
            rprintln!("SPI device {}: no response", i);
        }
        cs.set_high().ok();
    }
}
