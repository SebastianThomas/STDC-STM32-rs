use stm32l4xx_hal::{
    hal::{blocking::spi::Write, digital::v2::OutputPin},
    spi::Spi,
};

use crate::components::spi_utils::SpiError;

// Depth fields (e.g. "30.0m") need a few chars; keep small but safe
const STATUS_DEPTH_CACHE_LEN: usize = 8;
// Minute fields are fixed-width D=3 (right-aligned), so use 3 bytes
const STATUS_MINUTES_LEN: usize = 3;
const STATUS_LABEL_CACHE_LEN: usize = STATUS_MINUTES_LEN;

pub struct SpiDisplay<SPI, PINS, EN: OutputPin, RST: OutputPin, NDC: OutputPin> {
    power_enable: EN,
    spi: Spi<SPI, PINS>,
    spi_reset: RST,
    not_data_command: NDC,

    enabled: bool,
    depth_cache: [u8; STATUS_DEPTH_CACHE_LEN],
    depth_cache_len: usize,
    time_cache: [u8; STATUS_MINUTES_LEN],
    time_cache_len: usize,
    stop_time_cache: [u8; STATUS_MINUTES_LEN],
    stop_time_cache_len: usize,
    deco_cache: [u8; STATUS_DEPTH_CACHE_LEN],
    deco_cache_len: usize,
    tts_cache: [u8; STATUS_MINUTES_LEN],
    tts_cache_len: usize,
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
            depth_cache: [0; STATUS_DEPTH_CACHE_LEN],
            depth_cache_len: 0,
            time_cache: [0; STATUS_MINUTES_LEN],
            time_cache_len: 0,
            stop_time_cache: [0; STATUS_MINUTES_LEN],
            stop_time_cache_len: 0,
            deco_cache: [0; STATUS_DEPTH_CACHE_LEN],
            deco_cache_len: 0,
            tts_cache: [0; STATUS_MINUTES_LEN],
            tts_cache_len: 0,
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

    pub(crate) fn update_depth_cache(&mut self, value: &[u8]) -> bool {
        self.update_cache(true, value)
    }

    pub(crate) fn update_time_cache(&mut self, value: &[u8]) -> bool {
        self.update_cache(false, value)
    }

    pub(crate) fn depth_cache_len(&self) -> usize {
        self.depth_cache_len
    }

    pub(crate) fn time_cache_len(&self) -> usize {
        self.time_cache_len
    }

    pub(crate) fn deco_cache_len(&self) -> usize {
        self.deco_cache_len
    }

    pub(crate) fn stop_time_cache_len(&self) -> usize {
        self.stop_time_cache_len
    }

    pub(crate) fn tts_cache_len(&self) -> usize {
        self.tts_cache_len
    }

    pub(crate) fn update_deco_cache(&mut self, value: &[u8]) -> bool {
        self.update_deco_or_tts_cache(true, value)
    }

    pub(crate) fn update_tts_cache(&mut self, value: &[u8]) -> bool {
        self.update_deco_or_tts_cache(false, value)
    }

    pub(crate) fn update_stop_time_cache(&mut self, value: &[u8]) -> bool {
        self.update_stop_time_cache_inner(value)
    }

    fn update_cache(&mut self, is_depth: bool, value: &[u8]) -> bool {
        let len = if is_depth {
            core::cmp::min(value.len(), STATUS_DEPTH_CACHE_LEN)
        } else {
            core::cmp::min(value.len(), STATUS_MINUTES_LEN)
        };
        let (cache, cache_len) = if is_depth {
            (&mut self.depth_cache[..], &mut self.depth_cache_len)
        } else {
            (&mut self.time_cache[..], &mut self.time_cache_len)
        };

        if *cache_len == len && cache[..len] == value[..len] {
            return false;
        }

        cache[..len].copy_from_slice(&value[..len]);
        *cache_len = len;
        true
    }

    fn update_deco_or_tts_cache(&mut self, is_deco: bool, value: &[u8]) -> bool {
        let len = if is_deco {
            core::cmp::min(value.len(), STATUS_DEPTH_CACHE_LEN)
        } else {
            core::cmp::min(value.len(), STATUS_MINUTES_LEN)
        };
        let (cache, cache_len) = if is_deco {
            (&mut self.deco_cache[..], &mut self.deco_cache_len)
        } else {
            (&mut self.tts_cache[..], &mut self.tts_cache_len)
        };

        if *cache_len == len && cache[..len] == value[..len] {
            return false;
        }

        cache[..len].copy_from_slice(&value[..len]);
        *cache_len = len;
        true
    }

    fn update_stop_time_cache_inner(&mut self, value: &[u8]) -> bool {
        let len = core::cmp::min(value.len(), STATUS_MINUTES_LEN);

        if self.stop_time_cache_len == len && self.stop_time_cache[..len] == value[..len] {
            return false;
        }

        self.stop_time_cache[..len].copy_from_slice(&value[..len]);
        self.stop_time_cache_len = len;
        true
    }
}

impl<SPI, PINS, EN: OutputPin, RST: OutputPin, NDC: OutputPin> SpiDisplay<SPI, PINS, EN, RST, NDC>
where
    Spi<SPI, PINS>: Write<u8>,
{
    pub(crate) fn write_data_raw(&mut self, data: &[u8]) -> Result<(), SpiError> {
        Write::write(&mut self.spi, data)
            .map_err(|_| SpiError::new(1, "Failed writing display data over SPI"))
    }

    pub(crate) fn write_command(&mut self, command: u8) -> Result<(), SpiError> {
        self.set_command_mode()?;
        self.write_data_raw(&[command])
    }

    pub(crate) fn write_data(&mut self, data: &[u8]) -> Result<(), SpiError> {
        self.set_data_mode()?;
        self.write_data_raw(data)
    }
}
