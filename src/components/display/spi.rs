use stm32l4xx_hal::{
    hal::{blocking::spi::Write, digital::v2::OutputPin},
    spi::Spi,
};

use crate::components::spi_utils::SpiError;

pub struct SpiDisplay<SPI, PINS, EN: OutputPin, RST: OutputPin, NDC: OutputPin> {
    power_enable: EN,
    spi: Spi<SPI, PINS>,
    spi_reset: RST,
    not_data_command: NDC,

    enabled: bool,
}

impl<SPI, PINS, EN: OutputPin, RST: OutputPin, NDC: OutputPin> SpiDisplay<SPI, PINS, EN, RST, NDC> {
    pub fn new(
        power_enable: EN,
        spi: Spi<SPI, PINS>,
        spi_reset: RST,
        not_data_command: NDC,
    ) -> SpiDisplay<SPI, PINS, EN, RST, NDC> {
        SpiDisplay {
            power_enable,
            spi,
            spi_reset,
            not_data_command,
            enabled: false,
        }
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    pub fn shutoff(&mut self) -> Result<bool, SpiError> {
        if !self.enabled {
            return Ok(false);
        }
        self.power_enable
            .set_low()
            .map_err(|_| SpiError::new(1, "Failed setting display enable pin low"))?;
        self.enabled = false;
        Ok(true)
    }

    pub fn turn_on(&mut self) -> Result<bool, SpiError> {
        if self.enabled {
            return Ok(false);
        }
        self.power_enable
            .set_high()
            .map_err(|_| SpiError::new(1, "Failed setting display enable pin high"))?;
        self.enabled = true;
        Ok(true)
    }

    pub fn reset(&mut self) -> Result<(), SpiError> {
        self.spi_reset
            .set_low()
            .map_err(|_| SpiError::new(1, "Failed setting display reset pin low"))?;
        self.spi_reset
            .set_high()
            .map_err(|_| SpiError::new(1, "Failed setting display reset pin high"))?;
        Ok(())
    }

    pub fn reset_with_delay<F>(&mut self, mut delay: F) -> Result<(), SpiError>
    where
        F: FnMut(),
    {
        self.spi_reset
            .set_low()
            .map_err(|_| SpiError::new(1, "Failed setting display reset pin low"))?;
        delay();
        self.spi_reset
            .set_high()
            .map_err(|_| SpiError::new(1, "Failed setting display reset pin high"))?;
        delay();
        Ok(())
    }

    pub(crate) fn set_command_mode(&mut self) -> Result<(), SpiError> {
        self.not_data_command
            .set_low()
            .map_err(|_| SpiError::new(1, "Failed setting display D/C to command mode"))
    }

    pub(crate) fn set_data_mode(&mut self) -> Result<(), SpiError> {
        self.not_data_command
            .set_high()
            .map_err(|_| SpiError::new(1, "Failed setting display D/C to data mode"))
    }
}

impl<SPI, PINS, EN: OutputPin, RST: OutputPin, NDC: OutputPin> SpiDisplay<SPI, PINS, EN, RST, NDC>
where
    Spi<SPI, PINS>: Write<u8>,
{
    pub(crate) fn write_command(&mut self, command: u8) -> Result<(), SpiError> {
        self.set_command_mode()?;
        Write::write(&mut self.spi, &[command])
            .map_err(|_| SpiError::new(1, "Failed writing display command over SPI"))
    }

    pub(crate) fn write_data(&mut self, data: &[u8]) -> Result<(), SpiError> {
        self.set_data_mode()?;
        Write::write(&mut self.spi, data)
            .map_err(|_| SpiError::new(1, "Failed writing display data over SPI"))
    }
}
