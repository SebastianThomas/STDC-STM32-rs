use crate::components::flash::Flash;

pub const FIRMWARE_VERSION: u16 = 0;
pub const COMPUTER_SERIAL_NUMBER: u8 = 0;
pub const LOG_MODEL_VERSION: u8 = 0;
pub const SALINITY: Salinity = Salinity::FRESH;
pub const DIVE_MODE: DiveMode = DiveMode::OC;
pub const DECO_ALGORITHM: DecoAlgorithm = DecoAlgorithm::Exponential;
pub const GF_LOW: u8 = 50;
pub const GF_HIGH: u8 = 75;

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Salinity {
    FRESH,
    EN13319,
    SALT,
}

impl Salinity {
    pub fn density_f32(&self) -> f32 {
        match self {
            Salinity::FRESH => 1000.0,
            Salinity::EN13319 => 1020.0,
            Salinity::SALT => 1035.0,
        }
    }
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DiveMode {
    OC,
    CC,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DecoAlgorithm {
    Exponential,
    LinearExponential,
}

pub struct LogDiveControlDataBlock<const GAS_NR: u8>
where
    [(); GAS_NR as usize * 3]:,
{
    pub firmware_version: u16,
    pub computer_serial_nr: u8,
    pub log_model_version: u8,
    pub time_offset: u32,
    pub surface_interval: u32,
    pub dive_number: u16,
    pub surface_pressure: u16,
    pub surface_temperature: i8,
    pub ascent_rate_agg: u8,
    pub salinity: Salinity,
    pub dive_mode: DiveMode,
    pub deco_algorithm: DecoAlgorithm,
    pub gf_low: u8,
    pub gf_high: u8,
    pub gas_nr: u8,
    pub gas_content: [u8; GAS_NR as usize * 3],
}

impl<const GAS_NR: u8> LogDiveControlDataBlock<GAS_NR>
where
    [(); 4 + 24 + GAS_NR as usize * 3]:,
    [(); 24 + GAS_NR as usize * 3]: Sized,
{
    // const LEN: usize = 24 + GAS_NR as usize * 3;

    pub fn new(
        start_epoch_seconds: u32,
        surface_interval_seconds: u32,
        dive_number: u16,
        surface_pressure_e2_pa: u16,
        surface_temperature: f32,
        ascent_rate_agg_seconds: u8,
        gas_content: [u8; GAS_NR as usize * 3],
    ) -> LogDiveControlDataBlock<GAS_NR> {
        LogDiveControlDataBlock {
            firmware_version: FIRMWARE_VERSION,
            computer_serial_nr: COMPUTER_SERIAL_NUMBER,
            log_model_version: LOG_MODEL_VERSION,
            time_offset: start_epoch_seconds,
            surface_interval: surface_interval_seconds,
            dive_number,
            surface_pressure: surface_pressure_e2_pa,
            surface_temperature: (surface_temperature * 2.0) as i8,
            ascent_rate_agg: ascent_rate_agg_seconds,
            salinity: SALINITY,
            dive_mode: DIVE_MODE,
            deco_algorithm: DECO_ALGORITHM,
            gf_low: GF_LOW,
            gf_high: GF_HIGH,
            gas_nr: GAS_NR,
            gas_content,
        }
    }

    pub fn write<F: Flash>(&self, flash: &mut F) -> Result<u32, F::Error>
    where
        [(); 4 + (24 + GAS_NR as usize * 3)]:,
        [(); 4 + (24 + GAS_NR as usize * 3) + 0]: Sized,
        [(); 24 + GAS_NR as usize * 3 + 0]: Sized,
    {
        let mut bytes: [u8; 24 + GAS_NR as usize * 3] = [0; 24 + GAS_NR as usize * 3];
        bytes[0] = (self.firmware_version >> 8 & 0xFF) as u8;
        bytes[1] = (self.firmware_version & 0xFF) as u8;
        bytes[2] = self.computer_serial_nr;
        bytes[3] = self.log_model_version;
        bytes[4] = (self.time_offset >> 24 & 0xFF) as u8;
        bytes[5] = (self.time_offset >> 16 & 0xFF) as u8;
        bytes[6] = (self.time_offset >> 8 & 0xFF) as u8;
        bytes[7] = (self.time_offset & 0xFF) as u8;
        bytes[8] = (self.surface_interval >> 24 & 0xFF) as u8;
        bytes[9] = (self.surface_interval >> 16 & 0xFF) as u8;
        bytes[10] = (self.surface_interval >> 8 & 0xFF) as u8;
        bytes[11] = (self.surface_interval & 0xFF) as u8;
        bytes[12] = (self.dive_number >> 8 & 0xFF) as u8;
        bytes[13] = (self.dive_number & 0xFF) as u8;
        bytes[14] = (self.surface_pressure >> 8 & 0xFF) as u8;
        bytes[15] = (self.surface_pressure & 0xFF) as u8;
        bytes[16] = (self.surface_temperature & 0xFFu8 as i8) as u8;
        bytes[17] = self.ascent_rate_agg;
        bytes[18] = self.salinity as u8;
        bytes[19] = self.dive_mode as u8;
        bytes[20] = self.deco_algorithm as u8;
        bytes[21] = self.gf_low;
        bytes[22] = self.gf_high;
        bytes[23] = self.gas_nr;

        bytes[24..=24 + GAS_NR as usize * 3 - 1].copy_from_slice(&self.gas_content);

        flash.write::<{ 24 + GAS_NR as usize * 3 }>(&bytes)
    }
}

#[repr(u8)]
pub enum LevelState {
    Level,
    Ascending,
    Descending,
    Error,
}

impl LevelState {
    pub fn from_bits(bits: u8) -> Self {
        match bits & 0b11 {
            0 => LevelState::Level,
            1 => LevelState::Ascending,
            2 => LevelState::Descending,
            3 => LevelState::Error,
            _ => unreachable!(),
        }
    }

    pub fn bits(self) -> u8 {
        self as u8 & 0b11
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LogPointMetadata {
    byte: u8,
}

impl LogPointMetadata {
    // bit positions
    const DIVE_MODE_BIT: u8 = 0;
    const DECO_OBLIGATION_BIT: u8 = 1;
    const LEVEL_STATE_SHIFT: u8 = 2;
    const OPTIONAL_PARAM_SHIFT: u8 = 4;

    pub fn new(
        dive_mode_active: bool,
        deco_obligation: bool,
        level_state: LevelState,
        optional_param_pages: [bool; 4],
    ) -> Self {
        let mut byte = 0u8;
        if dive_mode_active {
            byte |= 1 << Self::DIVE_MODE_BIT;
        }
        if deco_obligation {
            byte |= 1 << Self::DECO_OBLIGATION_BIT;
        }

        byte |= level_state.bits() << Self::LEVEL_STATE_SHIFT;

        for (i, &flag) in optional_param_pages.iter().enumerate() {
            if flag {
                byte |= 1 << (Self::OPTIONAL_PARAM_SHIFT + i as u8);
            }
        }

        LogPointMetadata { byte }
    }

    pub fn dive_mode_active(&self) -> bool {
        self.byte & (1 << Self::DIVE_MODE_BIT) != 0
    }

    pub fn deco_obligation(&self) -> bool {
        self.byte & (1 << Self::DECO_OBLIGATION_BIT) != 0
    }

    pub fn level_state(&self) -> LevelState {
        LevelState::from_bits((self.byte >> Self::LEVEL_STATE_SHIFT) & 0b11)
    }

    pub fn optional_param_pages(&self) -> [bool; 4] {
        let mut arr = [false; 4];
        for i in 0u8..4u8 {
            arr[i as usize] = (self.byte & (1 << (Self::OPTIONAL_PARAM_SHIFT + i))) != 0;
        }
        arr
    }
}

pub struct BasicLogPointData {
    time_delta: u16,
    depth: u16,
    metadata: LogPointMetadata,
    battery: u8,
    temperature: i8,
    ascent_rate: u8,
}

pub struct DecoLogPointData {
    ndl: u8,
    ceiling: u8,
    gf99: u8,
    leading_tissue_id: u8,
    risk: u8,
    gas_mix_id: u8,
    po2: u8,
    cns: u8,
}

pub struct LogPointData {
    basic_data: BasicLogPointData,
    deco_data: Option<DecoLogPointData>,
}

impl LogPointData {
    pub fn write<F: Flash>(&self, flash: &mut F) -> Result<(u32, Option<u32>), F::Error> {
        // Make sure metadata optional pages are always valid
        let metadata =
            self.basic_data.metadata.byte & if self.deco_data.is_some() { 0xF7 } else { 0xF7 };

        let basic_page: [u8; 8] = [
            (self.basic_data.time_delta >> 8 & 0xFF) as u8,
            (self.basic_data.time_delta & 0xFF) as u8,
            (self.basic_data.depth >> 8 & 0xFF) as u8,
            (self.basic_data.depth & 0xFF) as u8,
            metadata,
            self.basic_data.battery,
            self.basic_data.temperature as u8,
            self.basic_data.ascent_rate,
        ];
        let basic_start_addr = flash.write(&basic_page)?;
        if self.basic_data.metadata.optional_param_pages()[3]
            && let Some(deco_page) = &self.deco_data
        {
            let deco_page: [u8; 8] = [
                deco_page.ndl,
                deco_page.ceiling,
                deco_page.gf99,
                deco_page.leading_tissue_id,
                deco_page.risk,
                deco_page.gas_mix_id,
                deco_page.po2,
                deco_page.cns,
            ];
            let deco_start_addr = flash.write(&deco_page)?;
            return Ok((basic_start_addr, Some(deco_start_addr)));
        }
        Ok((basic_start_addr, None))
    }
}
