use core::fmt::{Display, Formatter};

pub trait DetailsError {
    fn details(&self) -> &'static str;
}

#[derive(Debug)]
pub struct SpiError {
    pub priority: u8,
    pub details: &'static str,
}

impl SpiError {
    pub fn new(priority: u8, details: &'static str) -> SpiError {
        SpiError { priority, details }
    }
}

impl DetailsError for SpiError {
    fn details(&self) -> &'static str {
        self.details
    }
}

impl Display for SpiError {
    fn fmt(&self, f: &mut Formatter) -> core::fmt::Result {
        write!(f, "SPI Flash Error: {}", self.details)
    }
}

impl core::error::Error for SpiError {}
