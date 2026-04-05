use fixed::types::U10F6;
use thalmann::{
    dive::DiveMeasurement,
    gas::{CCRGas, GasMix},
    pressure_unit::{AbsPressure, Pa, Pressure},
};

#[cfg(target_os = "none")]
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

pub enum CurrentDiveModeWithInfo {
    OC {
        gas_idx: usize,
    },
    CC {
        partial_pressure: Pa,
        dil_idx: usize,
    },
}

impl CurrentDiveModeWithInfo {
    pub fn to_log_byte(&self) -> u8 {
        const MSB_U8: u8 = 1 << 7;
        match self {
            Self::OC { gas_idx } => *gas_idx as u8 & !MSB_U8,
            Self::CC { dil_idx, .. } => MSB_U8 | (*dil_idx as u8) & !MSB_U8,
        }
    }

    pub fn to_fixed_gas<const NUM_GASES: usize, D: const AbsPressure>(
        &self,
        gases: &[GasMix<f32>; NUM_GASES],
        depth: D,
    ) -> GasMix<f32> {
        match self {
            CurrentDiveModeWithInfo::OC { gas_idx } => gases[*gas_idx],
            CurrentDiveModeWithInfo::CC {
                partial_pressure,
                dil_idx,
            } => CCRGas {
                diluent: gases[*dil_idx],
                set_point: *partial_pressure,
            }
            .to_fixed_gas_mix(depth),
        }
    }
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DecoAlgorithm {
    Exponential,
    LinearExponential,
}

pub struct LogDiveControlDataBlock<const GAS_NR: usize>
where
    [(); GAS_NR * 3]: Sized,
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
    pub gas_content: [u8; GAS_NR * 3],
}

fn get_gas_content<const GAS_NR: usize>(gas_content: &[GasMix<f32>; GAS_NR]) -> [u8; GAS_NR * 3]
where
    [(); GAS_NR * 3]: Sized,
{
    let mut buf = [0u8; GAS_NR * 3];
    for i in 0..GAS_NR {
        buf[i * 3] = gas_content[i].fo2() as u8;
        buf[i * 3 + 1] = gas_content[i].fn2() as u8;
        buf[i * 3 + 2] = gas_content[i].fhe() as u8;
    }
    buf
}

impl<const GAS_NR: usize> LogDiveControlDataBlock<GAS_NR>
where
    [(); 4 + 24 + GAS_NR * 3]: Sized,
    [(); 24 + GAS_NR * 3]: Sized,
    [(); GAS_NR * 3]: Sized,
{
    pub fn from_bytes(bytes: &[u8]) -> Option<(Self, usize)> {
        if bytes.len() < 24 {
            return None;
        }

        let gas_nr = bytes[23] as usize;
        if gas_nr > GAS_NR {
            return None;
        }

        let gas_content_len = gas_nr * 3;
        let total_len = 24 + gas_content_len;
        if bytes.len() < total_len {
            return None;
        }

        let mut gas_content = [0u8; GAS_NR * 3];
        if gas_content_len > 0 {
            gas_content[..gas_content_len].copy_from_slice(&bytes[24..total_len]);
        }

        Some((
            Self {
                firmware_version: u16::from_be_bytes([bytes[0], bytes[1]]),
                computer_serial_nr: bytes[2],
                log_model_version: bytes[3],
                time_offset: u32::from_be_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
                surface_interval: u32::from_be_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]),
                dive_number: u16::from_be_bytes([bytes[12], bytes[13]]),
                surface_pressure: u16::from_be_bytes([bytes[14], bytes[15]]),
                surface_temperature: bytes[16] as i8,
                ascent_rate_agg: bytes[17],
                salinity: match bytes[18] {
                    0 => Salinity::FRESH,
                    1 => Salinity::EN13319,
                    2 => Salinity::SALT,
                    _ => Salinity::FRESH,
                },
                dive_mode: match bytes[19] {
                    0 => DiveMode::OC,
                    1 => DiveMode::CC,
                    _ => DiveMode::OC,
                },
                deco_algorithm: match bytes[20] {
                    0 => DecoAlgorithm::Exponential,
                    1 => DecoAlgorithm::LinearExponential,
                    _ => DecoAlgorithm::Exponential,
                },
                gf_low: bytes[21],
                gf_high: bytes[22],
                gas_nr: bytes[23],
                gas_content,
            },
            total_len,
        ))
    }

    pub fn gas_content_len(&self) -> usize {
        self.gas_nr as usize * 3
    }

    pub fn gas_content_bytes(&self) -> &[u8] {
        &self.gas_content[..self.gas_content_len()]
    }

    pub fn firmware_version(&self) -> u16 {
        self.firmware_version
    }

    pub fn computer_serial_number(&self) -> u8 {
        self.computer_serial_nr
    }

    pub fn log_model_version(&self) -> u8 {
        self.log_model_version
    }

    pub fn time_offset(&self) -> u32 {
        self.time_offset
    }

    pub fn surface_interval(&self) -> u32 {
        self.surface_interval
    }

    pub fn dive_number(&self) -> u16 {
        self.dive_number
    }

    pub fn surface_pressure_hpa(&self) -> u16 {
        self.surface_pressure
    }

    pub fn surface_temperature_half_c(&self) -> i8 {
        self.surface_temperature
    }

    pub fn ascent_rate_agg_seconds(&self) -> u8 {
        self.ascent_rate_agg
    }

    pub fn salinity_value(&self) -> Salinity {
        self.salinity
    }

    pub fn dive_mode_value(&self) -> DiveMode {
        self.dive_mode
    }

    pub fn deco_algorithm_value(&self) -> DecoAlgorithm {
        self.deco_algorithm
    }

    pub fn gf_low_value(&self) -> u8 {
        self.gf_low
    }

    pub fn gf_high_value(&self) -> u8 {
        self.gf_high
    }

    pub fn raw_bytes(&self) -> [u8; 24 + GAS_NR * 3] {
        let mut bytes = [0u8; 24 + GAS_NR * 3];
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
        bytes[24..].copy_from_slice(&self.gas_content);
        bytes
    }

    pub fn new(
        start_epoch_seconds: u32,
        surface_interval_seconds: u32,
        dive_number: u16,
        surface_pressure_hpa: u16,
        surface_temperature: i8,
        ascent_rate_agg_seconds: u8,
        gas_content: &[GasMix<f32>; GAS_NR],
    ) -> Self
    where
        [(); GAS_NR * 3]: Sized,
    {
        LogDiveControlDataBlock {
            firmware_version: FIRMWARE_VERSION,
            computer_serial_nr: COMPUTER_SERIAL_NUMBER,
            log_model_version: LOG_MODEL_VERSION,
            time_offset: start_epoch_seconds,
            surface_interval: surface_interval_seconds,
            dive_number,
            surface_pressure: surface_pressure_hpa,
            surface_temperature: surface_temperature * 2,
            ascent_rate_agg: ascent_rate_agg_seconds,
            salinity: SALINITY,
            dive_mode: DIVE_MODE,
            deco_algorithm: DECO_ALGORITHM,
            gf_low: GF_LOW,
            gf_high: GF_HIGH,
            gas_nr: GAS_NR as u8,
            gas_content: get_gas_content(gas_content),
        }
    }

    pub fn surface_temperature_celsius(&self) -> f32 {
        self.surface_temperature as f32 / 2.0
    }

    pub fn gas_content(&self) -> [GasMix<f32>; GAS_NR] {
        core::array::from_fn(|i| {
            let base = i * 3;
            let o2 = self.gas_content[base] as f32;
            let he = self.gas_content[base + 2] as f32;
            match GasMix::new(o2, he) {
                Ok(gas) => gas,
                Err(_) => GasMix::new(0.0, 0.0).unwrap(),
            }
        })
    }

    #[cfg(target_os = "none")]
    pub fn write<F: Flash>(&self, flash: &mut F) -> Result<u32, F::Error>
    where
        [(); 4 + (24 + GAS_NR * 3)]: Sized,
        [(); 4 + (24 + GAS_NR * 3) + 0]: Sized,
        [(); 24 + GAS_NR * 3 + 0]: Sized,
    {
        let mut bytes: [u8; 24 + GAS_NR * 3] = [0; 24 + GAS_NR * 3];
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

        bytes[24..=24 + GAS_NR * 3 - 1].copy_from_slice(&self.gas_content);

        flash.write::<{ 24 + GAS_NR * 3 }>(&bytes)
    }
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
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

    pub fn byte(&self) -> u8 {
        self.byte
    }
}

pub struct BasicLogPointData {
    time_delta: u16,
    depth_m: U10F6,
    metadata: LogPointMetadata,
    battery: u8,
    temperature: i8,
    ascent_rate: u8,
}

impl BasicLogPointData {
    pub fn time_delta_seconds(&self) -> u16 {
        self.time_delta
    }

    pub fn depth(&self) -> U10F6 {
        self.depth_m
    }

    pub fn metadata(&self) -> LogPointMetadata {
        self.metadata
    }

    pub fn battery(&self) -> u8 {
        self.battery
    }

    pub fn temperature(&self) -> i8 {
        self.temperature
    }

    pub fn ascent_rate(&self) -> u8 {
        self.ascent_rate
    }

    pub fn to_bytes(&self) -> [u8; 8] {
        [
            (self.time_delta >> 8 & 0xFF) as u8,
            (self.time_delta & 0xFF) as u8,
            (self.depth_m.to_bits() >> 8 & 0xFF) as u8,
            (self.depth_m.to_bits() & 0xFF) as u8,
            self.metadata.byte(),
            self.battery,
            self.temperature as u8,
            self.ascent_rate,
        ]
    }
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

impl DecoLogPointData {
    pub fn to_bytes(&self) -> [u8; 8] {
        [
            self.ndl,
            self.ceiling,
            self.gf99,
            self.leading_tissue_id,
            self.risk,
            self.gas_mix_id,
            self.po2,
            self.cns,
        ]
    }
}

pub struct LogPointData {
    basic_data: BasicLogPointData,
    deco_data: Option<DecoLogPointData>,
}

impl LogPointData {
    pub fn from_bytes(bytes: &[u8]) -> Option<(Self, usize)> {
        if bytes.len() < 8 {
            return None;
        }

        let has_deco = (bytes[4] & (1 << 7)) != 0;
        let basic_data = BasicLogPointData {
            time_delta: u16::from_be_bytes([bytes[0], bytes[1]]),
            depth_m: U10F6::from_bits(u16::from_be_bytes([bytes[2], bytes[3]])),
            metadata: LogPointMetadata { byte: bytes[4] },
            battery: bytes[5],
            temperature: bytes[6] as i8,
            ascent_rate: bytes[7],
        };

        let deco_data = if has_deco {
            if bytes.len() < 16 {
                return None;
            }

            Some(DecoLogPointData {
                ndl: bytes[8],
                ceiling: bytes[9],
                gf99: bytes[10],
                leading_tissue_id: bytes[11],
                risk: bytes[12],
                gas_mix_id: bytes[13],
                po2: bytes[14],
                cns: bytes[15],
            })
        } else {
            None
        };

        Some((
            Self {
                basic_data,
                deco_data,
            },
            if has_deco { 16 } else { 8 },
        ))
    }

    pub fn basic_data(&self) -> &BasicLogPointData {
        &self.basic_data
    }

    pub fn deco_data(&self) -> Option<&DecoLogPointData> {
        self.deco_data.as_ref()
    }

    pub fn basic_bytes(&self) -> [u8; 8] {
        self.basic_data.to_bytes()
    }

    pub fn deco_bytes(&self) -> Option<[u8; 8]> {
        self.deco_data.as_ref().map(DecoLogPointData::to_bytes)
    }
}

impl LogPointData {
    pub fn new<P: const AbsPressure>(
        metadata: LogPointMetadata,
        value: &DiveMeasurement<P>,
        ascent_rate: u8,
        temperature: i8,
        battery: u8,
    ) -> Self {
        let basic_data = BasicLogPointData {
            time_delta: (value.time_ms / 1000) as u16,
            depth_m: U10F6::from_num(value.depth.to_msw().to_f32()),
            metadata,
            battery,
            temperature,
            ascent_rate,
        };
        Self {
            basic_data,
            deco_data: None,
        }
    }

    #[cfg(target_os = "none")]
    pub fn write<F: Flash>(&self, flash: &mut F) -> Result<(u32, Option<u32>), F::Error> {
        // Make sure metadata optional pages are always valid
        let metadata =
            self.basic_data.metadata.byte & if self.deco_data.is_some() { 0xF7 } else { 0xF7 };

        let basic_page: [u8; 8] = [
            (self.basic_data.time_delta >> 8 & 0xFF) as u8,
            (self.basic_data.time_delta & 0xFF) as u8,
            (self.basic_data.depth_m.to_bits() >> 8 & 0xFF) as u8,
            (self.basic_data.depth_m.to_bits() & 0xFF) as u8,
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
