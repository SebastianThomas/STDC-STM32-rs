use core::cell::RefCell;

use cortex_m::interrupt::Mutex as CmMutex;
use stm32l4xx_hal::{delay::Delay, i2c::I2c, pac, spi::Spi};

use stdc_stm32_rs::{
    components::{
        MS5849,
        battery_status::{BatterySnapshot, BatteryStatusI2C},
        bluetooth::UartBluetoothModule,
        display::SpiDisplay,
        flash::SpiFlash,
        uart_log::UartLogger,
    },
    constants::barometric::SURFACE_PA,
};
use thalmann::pressure_unit::{Pa, msw};

pub type DelayMutex = CmMutex<RefCell<Delay>>;

pub type DebugTx = stm32l4xx_hal::serial::Tx<pac::USART1>;
pub type DebugRx = stm32l4xx_hal::serial::Rx<pac::USART1>;
pub type DebugLogger = UartLogger<DebugTx, DebugRx>;
pub type LoggerMutex = CmMutex<RefCell<DebugLogger>>;

pub type Pb6I2c1Scl = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Alternate<stm32l4xx_hal::gpio::OpenDrain, 4>,
    stm32l4xx_hal::gpio::L8,
    'B',
    6,
>;
pub type Pb7I2c1Sda = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Alternate<stm32l4xx_hal::gpio::OpenDrain, 4>,
    stm32l4xx_hal::gpio::L8,
    'B',
    7,
>;
pub type SensorI2c = I2c<pac::I2C1, (Pb6I2c1Scl, Pb7I2c1Sda)>;
pub type SensorMs5849<'a> = MS5849<'a, SensorI2c, ()>;

pub type Pc0I2c3Scl = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Alternate<stm32l4xx_hal::gpio::OpenDrain, 4>,
    stm32l4xx_hal::gpio::L8,
    'C',
    0,
>;
pub type Pc1I2c3Sda = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Alternate<stm32l4xx_hal::gpio::OpenDrain, 4>,
    stm32l4xx_hal::gpio::L8,
    'C',
    1,
>;
pub type BatteryI2c = I2c<pac::I2C3, (Pc0I2c3Scl, Pc1I2c3Sda)>;
pub type BatteryGauge<'a> = BatteryStatusI2C<'a, BatteryI2c>;

pub type Pc5Output = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Output<stm32l4xx_hal::gpio::PushPull>,
    stm32l4xx_hal::gpio::L8,
    'C',
    5,
>;
pub type Pc8Output = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Output<stm32l4xx_hal::gpio::OpenDrain>,
    stm32l4xx_hal::gpio::H8,
    'C',
    8,
>;
pub type Pc9Output = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Output<stm32l4xx_hal::gpio::OpenDrain>,
    stm32l4xx_hal::gpio::H8,
    'C',
    9,
>;
pub type Pc4Output = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Output<stm32l4xx_hal::gpio::PushPull>,
    stm32l4xx_hal::gpio::L8,
    'C',
    4,
>;
pub type Pb0Output = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Output<stm32l4xx_hal::gpio::PushPull>,
    stm32l4xx_hal::gpio::L8,
    'B',
    0,
>;
pub type Pa5Spi1Sck = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Alternate<stm32l4xx_hal::gpio::PushPull, 5>,
    stm32l4xx_hal::gpio::L8,
    'A',
    5,
>;
pub type Pa6Spi1Miso = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Alternate<stm32l4xx_hal::gpio::PushPull, 5>,
    stm32l4xx_hal::gpio::L8,
    'A',
    6,
>;
pub type Pa7Spi1Mosi = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Alternate<stm32l4xx_hal::gpio::PushPull, 5>,
    stm32l4xx_hal::gpio::L8,
    'A',
    7,
>;
pub type DisplaySpiPins = (Pa5Spi1Sck, Pa6Spi1Miso, Pa7Spi1Mosi);
pub type DisplayDevice = SpiDisplay<pac::SPI1, DisplaySpiPins, Pc5Output, Pc4Output, Pb0Output>;

pub type Pb12Output = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Output<stm32l4xx_hal::gpio::PushPull>,
    stm32l4xx_hal::gpio::H8,
    'B',
    12,
>;
pub type Pb13Spi2Sck = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Alternate<stm32l4xx_hal::gpio::PushPull, 5>,
    stm32l4xx_hal::gpio::H8,
    'B',
    13,
>;
pub type Pb14Spi2Miso = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Alternate<stm32l4xx_hal::gpio::PushPull, 5>,
    stm32l4xx_hal::gpio::H8,
    'B',
    14,
>;
pub type Pb15Spi2Mosi = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Alternate<stm32l4xx_hal::gpio::PushPull, 5>,
    stm32l4xx_hal::gpio::H8,
    'B',
    15,
>;
pub type FlashSpiPins = (Pb13Spi2Sck, Pb14Spi2Miso, Pb15Spi2Mosi);
pub type FlashSpiBus = Spi<pac::SPI2, FlashSpiPins>;
pub type FlashDevice = SpiFlash<FlashSpiBus, Pb12Output, fn(&[u8]) -> ()>;

pub type Pb2InputPullDown = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Input<stm32l4xx_hal::gpio::PullDown>,
    stm32l4xx_hal::gpio::L8,
    'B',
    2,
>;
pub type Pb10Usart3Tx = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Alternate<stm32l4xx_hal::gpio::PushPull, 7>,
    stm32l4xx_hal::gpio::H8,
    'B',
    10,
>;
pub type Pb11Usart3Rx = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Alternate<stm32l4xx_hal::gpio::PushPull, 7>,
    stm32l4xx_hal::gpio::H8,
    'B',
    11,
>;
pub type BluetoothTx = stm32l4xx_hal::serial::Tx<pac::USART3>;
pub type BluetoothRx = stm32l4xx_hal::serial::Rx<pac::USART3>;
pub type BluetoothModule = UartBluetoothModule<BluetoothTx, BluetoothRx, Pb2InputPullDown>;

pub type SurfacePressure = thalmann::pressure_unit::Pa;
pub const DEFAULT_SURFACE_PRESSURE: SurfacePressure = SURFACE_PA;

#[derive(Clone, Copy, Debug)]
pub struct Timestamped<T> {
    pub value: T,
    pub taken_millis: u32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct LatestMeasurements {
    pub pressure: Option<Timestamped<Pa>>,
    pub depth: Option<Timestamped<msw>>,
    pub temperature_c: Option<Timestamped<f32>>,
    pub battery: Option<Timestamped<BatterySnapshot>>,
}

impl LatestMeasurements {
    pub const fn new() -> Self {
        Self {
            pressure: None,
            depth: None,
            temperature_c: None,
            battery: None,
        }
    }

    pub fn record_environment(
        &mut self,
        pressure: Pa,
        depth: Option<msw>,
        temperature_c: Option<f32>,
        taken_millis: u32,
    ) {
        self.pressure = Some(Timestamped {
            value: pressure,
            taken_millis,
        });
        self.depth = depth.map(|value| Timestamped {
            value,
            taken_millis,
        });
        self.temperature_c = temperature_c.map(|value| Timestamped {
            value,
            taken_millis,
        });
    }

    pub fn record_battery(&mut self, battery: BatterySnapshot, taken_millis: u32) {
        self.battery = Some(Timestamped {
            value: battery,
            taken_millis,
        });
    }

    pub fn temperature_for_log(&self) -> i8 {
        match self.temperature_c {
            Some(temperature_c) => {
                let scaled = temperature_c.value * 2.0;
                if scaled >= 0.0 {
                    (scaled + 0.5) as i8
                } else {
                    (scaled - 0.5) as i8
                }
            }
            None => 0,
        }
    }

    pub fn battery_for_log(&self) -> u8 {
        match self.battery {
            Some(snapshot) => {
                let percent = snapshot.value.state_of_charge_percent;
                if percent <= 0.0 {
                    0
                } else if percent >= 100.0 {
                    100
                } else {
                    (percent + 0.5) as u8
                }
            }
            None => 0,
        }
    }
}
