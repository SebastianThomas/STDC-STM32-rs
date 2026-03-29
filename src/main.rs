#![no_main]
#![no_std]
#![feature(const_trait_impl)]
#![feature(const_default)]
#![feature(generic_const_exprs)]

use panic_probe as _;

use core::{
    cell::{Ref, RefCell},
    fmt::Debug,
    time::Duration,
};

use cortex_m::interrupt::{Mutex, free};
use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};

use stm32l4xx_hal::{
    delay::Delay,
    hal::{
        blocking::i2c::{Read, Write, WriteRead},
        spi::MODE_0,
    },
    i2c::{self, I2c},
    pac::{self, TIM2},
    prelude::*,
    rtc::{Rtc, RtcClockSource, RtcConfig},
    serial::Serial,
    spi::Spi,
    timer::Timer,
};

use thalmann::{
    calc_deco_schedule,
    display_utils::{format_f32, show_duration},
    dive::{DiveMeasurement, DiveProfile, StopSchedule},
    gas::{GasMix, MAX_GAS_DENSITY, TissuesLoading},
    loadings_from_dive_profile,
    mptt::{NUM_TISSUES, TISSUES, XVAL_HE9_040_F32},
    pressure_unit::{Bar, Pa, Pressure, msw},
    thalmann::DecoSettings,
};

use stdc_stm32_rs::{
    algorithms::{
        helpers::datetime_to_epoch_seconds,
        rate_algorithm::{DynamicDiffAimdRateAlgorithm, FixedRateAlgorithm, RateAlgorithm},
    },
    barometric::DepthOrAltitude,
    components::{
        MS5849,
        bluetooth::UartBluetoothModule,
        display::{DisplayState, MAX_STOP_NUMS, SpiDisplay},
        dive_log::{
            CurrentDiveModeWithInfo, LevelState, LogDiveControlDataBlock, LogPointData,
            LogPointMetadata,
        },
        flash::{Flash, SpiFlash},
        spi_utils::DetailsError,
        uart_log::{ExternalLogger, UartLogger},
    },
    concat_any_bytes,
};

static DISPLAY_STATE: Mutex<RefCell<DisplayState>> =
    Mutex::new(RefCell::new(DisplayState::default()));
const ENABLE_BLUETOOTH: bool = true;
static SERIAL_NUMBER: [u8; 4] = [0, 0, 0, 0];
static BLUETOOTH_NAME: [u8; 8] = concat_any_bytes!(b"STDC", SERIAL_NUMBER);

// TODO: Initial Position
const INITIAL_FLASH_ADDRESS: u32 = 1 << 21;

#[entry]
fn main() -> ! {
    rtt_init_print!();

    // ---------- START BOOT ----------
    rprintln!("Booting..");

    // 0001: Configure Peripherals, Clocks
    let cp = cortex_m::Peripherals::take().unwrap();

    let dp = pac::Peripherals::take().unwrap();

    // Enable required power and flash interfaces
    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    dp.DBGMCU.cr.modify(|_, w| w.trace_ioen().set_bit());

    // let mut ahb1 = rcc.ahb1;
    let mut apb1r1 = rcc.apb1r1;
    // let mut apb1r2 = rcc.apb1r2;
    let mut ahb2 = rcc.ahb2;
    let mut apb2 = rcc.apb2;
    let mut gpioa = dp.GPIOA.split(&mut ahb2);
    let mut gpiob = dp.GPIOB.split(&mut ahb2);
    let mut gpioc = dp.GPIOC.split(&mut ahb2);

    // TODO: Configure clock to 24 MHz if required with .sysclk(24.MHz())
    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    Timer::free_running_tim2(dp.TIM2, clocks, 1.kHz(), false, &mut apb1r1);

    let rtc_config = RtcConfig::default().clock_config(RtcClockSource::LSE);
    let mut rtc: Rtc = Rtc::rtc(dp.RTC, &mut apb1r1, &mut rcc.bdcr, &mut pwr.cr1, rtc_config);
    // TODO:
    rtc.set_date_time(
        stm32l4xx_hal::datetime::Date {
            day: 1,
            date: 10,
            month: 11,
            year: 2025,
        },
        stm32l4xx_hal::datetime::Time {
            hours: 0,
            minutes: 0,
            seconds: 0,
            micros: 0,
            daylight_savings: false,
        },
    );
    let (_boot_date, _boot_time) = rtc.get_date_time();
    let _boot_ms: u32 = millis_tim2();

    // ---------- FINISH BOOT ----------

    // ---------- START SETUP Protocols ----------

    // JTAG Set up by default
    // PA13: JTMS-SWDIO
    let _tms = gpioa
        .pa13
        .into_alternate::<0>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    // PA14: JTCK-SWCLK
    let _tck = gpioa
        .pa14
        .into_alternate::<0>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    // PA15: JTDI
    let _tdi = gpioa
        .pa15
        .into_alternate::<0>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    // PB3: JTDO-TRACESWO
    let _tdo = gpiob
        .pb3
        .into_alternate::<0>(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

    // Delay using systick
    let delay: Mutex<RefCell<Delay>> = Mutex::new(RefCell::new(Delay::new(cp.SYST, clocks)));

    // LEDs
    let mut dive_mode_indicator = gpioc
        .pc8
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
    let mut red_indicator = gpioc
        .pc9
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);

    // USART1 Debugging
    let usart1_tx = gpioa
        .pa9
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let usart1_rx = gpioa
        .pa10
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
    let debug_uart = Serial::usart1(
        dp.USART1,
        (usart1_tx, usart1_rx),
        115_200.bps(),
        clocks,
        &mut apb2,
    );
    let (debug_tx, debug_rx) = debug_uart.split();
    let logger = UartLogger::new(debug_tx, debug_rx);
    let logger = Mutex::new(RefCell::new(logger));
    log_bytes(&logger, "Logger to UART set up".as_bytes());

    // Display SPI
    // TODO: What mode is this in
    let spi_reset = gpioc
        .pc4
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
    let display_en = gpioc
        .pc5
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
    let not_data_command = gpiob
        .pb0
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

    let mut cs = gpioa
        .pa4
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let sck =
        gpioa
            .pa5
            .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let mosi =
        gpioa
            .pa7
            .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    // MISO Only for type checker - not connected on PCB, no input possible
    let miso_nc =
        gpioa
            .pa6
            .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    cs.set_high(); // idle high
    let display_spi = Spi::spi1(
        dp.SPI1,
        (sck, miso_nc, mosi),
        MODE_0,
        1.MHz(),
        clocks,
        &mut apb2,
    );
    let mut display = SpiDisplay::new(display_en, display_spi, spi_reset, not_data_command);
    let _ = display.turn_on();

    // Flash SPI
    let mut flash_cs_nss = gpiob
        .pb12
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let sck =
        gpiob
            .pb13
            .into_alternate_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    let miso =
        gpiob
            .pb14
            .into_alternate_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    let mosi =
        gpiob
            .pb15
            .into_alternate_push_pull(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    flash_cs_nss.set_high(); // idle high
    let flash_spi = Spi::spi2(
        dp.SPI2,
        (sck, miso, mosi),
        MODE_0,
        1.MHz(),
        clocks,
        &mut apb1r1,
    );
    let mut flash = SpiFlash::new(0, 1 << 22, flash_spi, flash_cs_nss, |bytes: &[u8]| {
        log_bytes(&logger, bytes);
    });

    let cur_addr = flash
        .read::<4>(INITIAL_FLASH_ADDRESS)
        .map(u32::from_be_bytes);
    if cur_addr.is_err() || *cur_addr.as_ref().unwrap() == 0 {
        let new_pos = INITIAL_FLASH_ADDRESS + 4;
        let new_pos_u8 = new_pos.to_be_bytes();
        let _ = flash.write(&new_pos_u8);
        let _ = flash.set_pos(new_pos);
    } else {
        // Unwrap safe, above if allows err
        let _ = flash.set_pos(cur_addr.unwrap());
    }
    if let Err(flash_rst_err) = flash.set_pos(INITIAL_FLASH_ADDRESS) {
        log_bytes(&logger, b"Failed setting initial flash address position");
        log_bytes(&logger, flash_rst_err.details().as_bytes());
    }
    let _ = flash.write(&SERIAL_NUMBER);

    // Sensor I2C
    let scl =
        gpiob
            .pb6
            .into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sda =
        gpiob
            .pb7
            .into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);
    let sensor_i2c = I2c::i2c1(
        dp.I2C1,
        (scl, sda),
        i2c::Config::new(100.kHz(), clocks),
        &mut apb1r1,
    );
    let mut ms5849_i2c = MS5849::new_i2c(sensor_i2c, &delay, |bytes: &[u8]| {
        log_bytes(&logger, bytes);
    });

    // Battery Status I2C
    let scl =
        gpioc
            .pc0
            .into_alternate_open_drain(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);
    let sda =
        gpioc
            .pc1
            .into_alternate_open_drain(&mut gpioc.moder, &mut gpioc.otyper, &mut gpioc.afrl);
    let battery_status_i2c = I2c::i2c3(
        dp.I2C3,
        (scl, sda),
        i2c::Config::new(100.kHz(), clocks),
        &mut apb1r1,
    );

    // Bluetooth UART
    let bluetooth_tx_ind = gpiob
        .pb2
        .into_pull_down_input(&mut gpiob.moder, &mut gpiob.pupdr);
    let usart3_tx = gpiob
        .pb10
        .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    let usart3_rx = gpiob
        .pb11
        .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    let bluetooth_uart = Serial::usart3(
        dp.USART3,
        (usart3_tx, usart3_rx),
        115_200.bps(),
        clocks,
        &mut apb1r1,
    );
    let (bluetooth_tx, bluetooth_rx) = bluetooth_uart.split();
    let bluetooth = UartBluetoothModule {
        uart_tx: bluetooth_tx,
        uart_rx: bluetooth_rx,
        tx_ind: bluetooth_tx_ind,
    };
    if ENABLE_BLUETOOTH {
        let _bluetooth = bluetooth.initialize_bluetooth(&BLUETOOTH_NAME);
    }

    // ---------- FINISH SETUP Protocols ----------

    // Example: Toggle LEDs
    loop {
        red_indicator.toggle();
        dive_mode_indicator.set_high();
        free(|cs| delay.borrow(cs).borrow_mut().delay_ms(300u16));
        dive_mode_indicator.set_low();
        free(|cs| delay.borrow(cs).borrow_mut().delay_ms(200u16));
        break;
    }

    rprintln!("Starting main waiting loop");
    loop {
        let (surface_pressure, first_dive_pressure) =
            surface_interval_wait(&mut ms5849_i2c, &mut flash, &delay, &rtc, &logger);
        start_log_dive(
            &mut ms5849_i2c,
            &mut flash,
            &rtc,
            &logger,
            (surface_pressure + first_dive_pressure) / 2.0,
        );
    }
}

fn surface_interval_wait<
    I: Write + Read + WriteRead,
    LFn: Fn(&[u8]) -> (),
    F: Flash,
    L: ExternalLogger,
>(
    ms5849_i2c: &mut MS5849<'_, I, (), LFn>,
    flash: &mut F,
    delay: &Mutex<RefCell<Delay>>,
    rtc: &Rtc,
    logger: &Mutex<RefCell<L>>,
) -> (Pa, Pa)
where
    <I as Read>::Error: Debug,
    <I as Write>::Error: Debug,
    <I as WriteRead>::Error: Debug,
{
    let mut prev_surface_altitude = None;
    let mut flash_log_algorithm = FixedRateAlgorithm::new(60_000);
    let mut last_logged_millis = 0;
    loop {
        let pressure = match ms5849_i2c.measure_pressure_to_pa() {
            Some(p) => p,
            None => {
                rprintln!("Got error while measuring pressure");
                continue;
            }
        };
        let current_measurement_millis = millis_tim2();
        let next_flash_iter =
            flash_log_algorithm.next_iter(last_logged_millis, current_measurement_millis);
        let next_flash_log = next_flash_iter.map(|n| last_logged_millis + n);
        if let Ok(n) = next_flash_log
            && n >= current_measurement_millis
        {
            let temperature = 0;
            let battery = 0;
            match LogPointData::new(
                LogPointMetadata::new(false, false, LevelState::Error, [false; 4]),
                &DiveMeasurement {
                    time_ms: current_measurement_millis as usize,
                    depth: pressure,
                    gas: 0b01111111,
                },
                0,
                temperature,
                battery,
            )
            .write(flash)
            {
                Ok(_) => {
                    last_logged_millis = current_measurement_millis;
                }
                Err(e) => {
                    rprintln!("Got error writing new measurement to flash{:?}", e);
                }
            }
        }
        match ms5849_i2c.depth_or_altitude() {
            Ok(DepthOrAltitude::Depth { pressure, depth }) => {
                rprintln!(
                    "Starting Dive: Pressure: {:?}, depth: {:?}",
                    pressure,
                    depth
                );
                return (pressure, prev_surface_altitude.unwrap_or(pressure));
            }
            Ok(DepthOrAltitude::Altitude { pressure, altitude }) => {
                prev_surface_altitude = Some(pressure);
                rprintln!(
                    "Measuring Pressure: {:?}, altitude: {:?}",
                    pressure,
                    altitude
                )
            }
            Err(err) => rprintln!(
                "Got error while reading depth for pressure {:?}: {:?}.",
                pressure,
                err,
            ),
        }
        log_bytes(&logger, "Sleeping for 5000ms".as_bytes());
        free(|cs| delay.borrow(cs).borrow_mut().delay_ms(3000u16));
    }
}

fn start_log_dive<I: Write + Read + WriteRead, LFn: Fn(&[u8]) -> (), F: Flash, L: ExternalLogger>(
    ms5849_i2c: &mut MS5849<'_, I, (), LFn>,
    flash: &mut F,
    rtc: &Rtc,
    logger: &Mutex<RefCell<L>>,
    surface_pressure: Pa,
) where
    <I as Read>::Error: Debug,
    <I as Write>::Error: Debug,
    <I as WriteRead>::Error: Debug,
{
    let dive_start_millis = millis_tim2();

    let deco_settings = DecoSettings {
        gas_density_settings: thalmann::gas::GasDensitySettings::Limit {
            limit: MAX_GAS_DENSITY.to_pa(),
        },
        max_deco_po2: Bar::new(1.6).to_pa(),
    };

    let gases = [thalmann::gas::AIR];
    let dive_profile = DiveProfile {
        dive_id: 1,
        max_depth: surface_pressure,
        gases,
        measurements: [DiveMeasurement {
            depth: surface_pressure,
            time_ms: dive_start_millis as usize,
            gas: 0,
        }],
    };
    let mut loading =
        loadings_from_dive_profile(&TISSUES, &dive_profile, &XVAL_HE9_040_F32, surface_pressure);

    let mut current_gas_mode_idx: CurrentDiveModeWithInfo =
        CurrentDiveModeWithInfo::OC { gas_idx: 0 };

    let (start_date, start_time) = rtc.get_date_time();
    let start_epoch_seconds = datetime_to_epoch_seconds(start_date, start_time);
    let surface_interval_seconds = 0;
    let dive_number = 0;
    let surface_pressure_hpa = (surface_pressure.to_hpa().to_f32()) as u16;
    let surface_temperature_2 = 0 * 2;
    let ascent_rate_agg_seconds = 4;
    let gas_content = gases;
    let dive_control_data_block = LogDiveControlDataBlock::new(
        start_epoch_seconds,
        surface_interval_seconds,
        dive_number,
        surface_pressure_hpa,
        surface_temperature_2,
        ascent_rate_agg_seconds,
        &gas_content,
    );

    dive_start_and_loop(
        dive_start_millis,
        surface_pressure,
        dive_control_data_block,
        ms5849_i2c,
        &deco_settings,
        &gases,
        &mut loading,
        &mut current_gas_mode_idx,
        flash,
        logger,
    );
}

fn dive_start_and_loop<
    const NUM_GASES: usize,
    I: Write + Read + WriteRead,
    LFn: Fn(&[u8]) -> (),
    F: Flash,
    L: ExternalLogger,
>(
    dive_start_millis: u32,
    surface_pressure: Pa,
    dive_control_data_block: LogDiveControlDataBlock<NUM_GASES>,
    ms5849_i2c: &mut MS5849<'_, I, (), LFn>,
    deco_settings: &DecoSettings<Pa>,
    gases: &[GasMix<f32>; NUM_GASES],
    loading: &mut TissuesLoading<NUM_TISSUES, Pa>,
    current_gas_mode_idx: &mut CurrentDiveModeWithInfo,
    flash: &mut F,
    logger: &Mutex<RefCell<L>>,
) where
    [(); NUM_GASES * 3]: Sized,
    [(); 24 + NUM_GASES * 3]: Sized,
    [(); 4 + 24 + NUM_GASES * 3]: Sized,
    [(); 4 + (24 + NUM_GASES * 3)]: Sized,
    [(); 4 + (24 + NUM_GASES * 3) + 0]: Sized,
    [(); 24 + NUM_GASES * 3 + 0]: Sized,
    <I as Read>::Error: Debug,
    <I as Write>::Error: Debug,
    <I as WriteRead>::Error: Debug,
{
    if let Err(e) = dive_control_data_block.write(flash) {
        log_bytes(&logger, b"Failed writing dive control data block.");
        log_bytes(&logger, e.details().as_bytes());
    }
    let mut last_measurement_millis = dive_start_millis;
    let mut last_logged_millis = dive_start_millis;
    let mut last_logged_pressure = Pa::new(dive_control_data_block.surface_pressure as f32);

    let mut flash_log_algorithm = DynamicDiffAimdRateAlgorithm::new::<3, 4>(
        5000,
        20000,
        Bar::new(0.2).to_pa(),
        Bar::new(1.0).to_pa(),
        Bar::new(0.2).to_pa(),
    );

    let mut max_depth = surface_pressure;

    loop {
        let current_millis = millis_tim2_since(dive_start_millis);
        let duration_since_start =
            Duration::from_millis((current_millis - dive_start_millis) as u64);
        display_set_dive_time(duration_since_start);

        let measurement_millis = millis_tim2();
        ms5849_i2c.read_i2c();
        match ms5849_i2c.depth_or_altitude() {
            Ok(DepthOrAltitude::Depth { pressure, depth }) => {
                handle_depth_measurement(
                    &mut flash_log_algorithm,
                    flash,
                    pressure,
                    depth,
                    &mut last_measurement_millis,
                    measurement_millis,
                    &mut last_logged_millis,
                    &mut last_logged_pressure,
                    &mut max_depth,
                    loading,
                    gases,
                    &current_gas_mode_idx,
                );
            }
            Ok(DepthOrAltitude::Altitude { pressure, altitude }) => {
                rprintln!(
                    "Measuring Pressure: {:?}, altitude: {:?}",
                    pressure,
                    altitude
                );
            }
            Err((pressure, err)) => {
                rprintln!(
                    "Got error while reading depth for pressure {:?}: {:?}.",
                    pressure,
                    err
                );
            }
        };
        let stops = calc_deco_schedule(loading, &gases, &deco_settings);
        match stops {
            Ok(stops) => display_set_stop_schedule(stops),
            Err(err) => rprintln!("Got error while calculating deco schedule: {:?}.", err),
        }
        display_refresh();
    }
}

fn handle_depth_measurement<const NUM_GASES: usize, R: RateAlgorithm<Pa, Pa, u32>, F: Flash>(
    flash_log_algorithm: &mut R,
    flash: &mut F,
    pressure: Pa,
    depth: msw,
    last_measurement_millis: &mut u32,
    measurement_millis: u32,
    last_logged_millis: &mut u32,
    last_logged_pressure: &mut Pa,
    max_depth: &mut Pa,
    loading: &mut TissuesLoading<NUM_TISSUES, Pa>,
    gases: &[GasMix<f32>; NUM_GASES],
    current_gas_mode_idx: &CurrentDiveModeWithInfo,
) {
    rprintln!("Measuring Pressure: {:?}, depth: {:?}", pressure, depth);
    display_set_depth(depth);
    let measurement = DiveMeasurement {
        time_ms: measurement_millis as usize,
        depth: pressure,
        gas: current_gas_mode_idx.to_log_byte() as usize,
    };
    let time_delta = measurement_millis - *last_measurement_millis;
    *last_measurement_millis = measurement_millis;

    if *max_depth < pressure {
        *max_depth = pressure;
    }

    let current_gas = current_gas_mode_idx.to_fixed_gas(gases, pressure);
    loading.tick(
        time_delta.try_into().unwrap_or(u16::MAX),
        pressure,
        &current_gas,
    );

    let next_log_time_delta = flash_log_algorithm.next_iter(*last_logged_pressure, pressure);
    let next_iter_due_in = next_log_time_delta.map(|n| *last_logged_millis + n);
    match next_iter_due_in {
        Ok(next_iter) => {
            if measurement_millis > next_iter {
                // TODO:
                let deco_obligation = false;
                let level_state = LevelState::Error;
                let ascent_rate = 0;
                let temperature = 0;
                let battery = 0;
                let log_point_data = LogPointData::new(
                    LogPointMetadata::new(true, deco_obligation, level_state, [false; 4]),
                    &measurement,
                    ascent_rate,
                    temperature,
                    battery,
                );
                match log_point_data.write(flash) {
                    Ok(_) => {
                        *last_logged_millis = measurement_millis;
                        *last_logged_pressure = pressure;
                    }
                    Err(e) => {
                        rprintln!("Failed writing log point data to flash: {:?}", e);
                    }
                }
            }
        }
        Err(e) => {
            rprintln!("Failed getting next log time: {:?}", e);
        }
    }
}

fn display_set_depth<P: Pressure>(depth: P) {
    free(|cs| {
        let mut display_state = DISPLAY_STATE.borrow(cs).borrow_mut();
        display_state.depth = depth.to_msw();
    });
}

fn display_set_dive_time(time: Duration) {
    free(|cs| {
        let mut display_state = DISPLAY_STATE.borrow(cs).borrow_mut();
        display_state.dive_time = time;
    });
}

fn display_set_stop_schedule(stops: StopSchedule<MAX_STOP_NUMS>) {
    free(|cs| {
        let mut display_state = DISPLAY_STATE.borrow(cs).borrow_mut();
        display_state.stop_schedule = Ok(stops);
    });
}

fn display_refresh() {
    free(|cs| {
        let display_state = DISPLAY_STATE.borrow(cs).borrow();
        display_refresh_with_state(display_state);
    });
}

fn display_refresh_with_state(display_state: Ref<DisplayState>) {
    let _duration_chars = show_duration(display_state.dive_time);
    let _meters_chars = format_f32::<' ', 3, 1>(display_state.depth.to_f32());
}

fn log_bytes<L: ExternalLogger>(logger: &Mutex<RefCell<L>>, bytes: &[u8]) {
    free(|cs| {
        if let Err(_) = logger.borrow(cs).borrow_mut().log_bytes(bytes) {
            rprintln!("Failed logging bytes to UART");
        }
    });
}

fn millis_tim2() -> u32 {
    Timer::<TIM2>::count()
}

fn millis_tim2_since(start: u32) -> u32 {
    Timer::<TIM2>::count().wrapping_sub(start)
}
