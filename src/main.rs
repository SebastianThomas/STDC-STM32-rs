#![no_main]
#![no_std]
#![feature(const_trait_impl)]
#![feature(const_default)]

use panic_probe as _;

use core::{
    cell::{Ref, RefCell},
    time::Duration,
};

use cortex_m::interrupt::{Mutex, free};
use cortex_m_rt::entry;
use rtt_target::{rprintln, rtt_init_print};

use stm32l4xx_hal::{
    delay::Delay,
    hal::spi::MODE_0,
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
    loadings_from_dive_profile,
    mptt::{TISSUES, XVAL_HE9_040_F32},
    pressure_unit::Pressure,
};

use stdc_stm32_rs::{
    barometric::{DepthOrAltitude, SURFACE_PA},
    components::{
        MS5849,
        display::{DisplayState, MAX_STOP_NUMS, SpiDisplay},
        flash::{Flash, SpiFlash},
        uart_log::{ExternalLogger, UartLogger},
    },
};

static DISPLAY_STATE: Mutex<RefCell<DisplayState>> =
    Mutex::new(RefCell::new(DisplayState::default()));
static SERIAL_NUMBER: [u8; 4] = [0, 0, 0, 0];

// TODO: Initial Position
const INITIAL_FLASH_ADDRESS: u32 = 1 << 21;

#[entry]
fn main() -> ! {
    rtt_init_print!();

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

    // ---------- SETUP Protocols ----------

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
    if let Err(flash_rst_err) = flash.set_pos(INITIAL_FLASH_ADDRESS) {
        log_bytes(&logger, flash_rst_err.details.as_bytes());
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
        let pressure = ms5849_i2c.measure_pressure_to_bar();
        match ms5849_i2c.depth_or_altitude() {
            Ok(DepthOrAltitude::Depth { pressure, depth }) => {
                rprintln!(
                    "Starting Dive: Pressure: {:?}, depth: {:?}",
                    pressure,
                    depth
                );
                break;
            }
            Ok(DepthOrAltitude::Altitude { pressure, altitude }) => rprintln!(
                "Measuring Pressure: {:?}, altitude: {:?}",
                pressure,
                altitude
            ),
            Err(err) => rprintln!(
                "Got error while reading depth for pressure {:?}: {:?}.",
                pressure,
                err,
            ),
        };
    }

    let dive_start_millis = millis_tim2();

    let gases = [thalmann::gas::AIR];
    let surface = SURFACE_PA;
    let dive_profile = DiveProfile {
        dive_id: 1,
        max_depth: surface,
        gases,
        measurements: [DiveMeasurement {
            depth: surface.to_pa(),
            time_ms: dive_start_millis as usize,
            gas: 0,
        }],
    };
    let loading = loadings_from_dive_profile(&TISSUES, &dive_profile, &XVAL_HE9_040_F32, surface);

    let current_gas_idx: usize = 0;

    loop {
        let current_millis = millis_tim2_since(dive_start_millis);
        let duration_since_start =
            Duration::from_millis((current_millis - dive_start_millis) as u64);
        set_dive_time(duration_since_start);

        let measurement_millis = millis_tim2();
        ms5849_i2c.read_i2c();
        match ms5849_i2c.depth_or_altitude() {
            Ok(DepthOrAltitude::Depth { pressure, depth }) => {
                rprintln!("Measuring Pressure: {:?}, depth: {:?}", pressure, depth);
                set_depth(depth);
                let measurement = DiveMeasurement {
                    gas: current_gas_idx,
                    depth: pressure,
                    time_ms: measurement_millis as usize,
                };
                // TODO: Add Measurement to Dive Profile
                // TODO: Update loading
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
        let stops = calc_deco_schedule(&loading, &gases);
        match stops {
            Ok(stops) => set_stop_schedule(stops),
            Err(err) => rprintln!("Got error while calculating deco schedule: {:?}.", err),
        }
        refresh_display();
    }
}

fn set_depth<P: Pressure>(depth: P) {
    free(|cs| {
        let mut display_state = DISPLAY_STATE.borrow(cs).borrow_mut();
        display_state.depth = depth.to_msw();
    });
}

fn set_dive_time(time: Duration) {
    free(|cs| {
        let mut display_state = DISPLAY_STATE.borrow(cs).borrow_mut();
        display_state.dive_time = time;
    });
}

fn set_stop_schedule(stops: StopSchedule<MAX_STOP_NUMS>) {
    free(|cs| {
        let mut display_state = DISPLAY_STATE.borrow(cs).borrow_mut();
        display_state.stop_schedule = Ok(stops);
    });
}

fn refresh_display() {
    free(|cs| {
        let display_state = DISPLAY_STATE.borrow(cs).borrow();
        refresh_display_with_state(display_state);
    });
}

fn refresh_display_with_state(display_state: Ref<DisplayState>) {
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
