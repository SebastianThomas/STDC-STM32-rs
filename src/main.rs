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
    pac::{self, TIM2},
    prelude::*,
    rtc::{Rtc, RtcConfig},
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
    MS5849,
    barometric::{DepthOrAltitude, SURFACE_PA},
    display::{DisplayState, MAX_STOP_NUMS},
};

static DISPLAY_STATE: Mutex<RefCell<DisplayState>> =
    Mutex::new(RefCell::new(DisplayState::default()));

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

    let mut ahb2 = rcc.ahb2;
    let mut apb2 = rcc.apb2;
    let mut gpioa = dp.GPIOA.split(&mut ahb2);
    let mut gpiob = dp.GPIOB.split(&mut ahb2);
    let mut gpioc = dp.GPIOC.split(&mut ahb2);

    // TODO: Configure clock to 24 MHz if required with .sysclk(24.MHz())
    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    Timer::free_running_tim2(dp.TIM2, clocks, 1.kHz(), false, &mut rcc.apb1r1);

    let rtc_config = RtcConfig::default(); // .clock_config(RtcClockSource::LSE);
    let mut rtc: Rtc = Rtc::rtc(
        dp.RTC,
        &mut rcc.apb1r1,
        &mut rcc.bdcr,
        &mut pwr.cr1,
        rtc_config,
    );
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

    // Custom SHIELD LEDs, TODO: remove
    let mut led1 = gpiob
        .pb6
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut led2 = gpioc
        .pc7
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);

    // Delay using systick
    let delay: Mutex<RefCell<Delay>> = Mutex::new(RefCell::new(Delay::new(cp.SYST, clocks)));

    // Example: Toggle LEDs
    loop {
        led1.toggle();
        led2.set_high();
        free(|cs| delay.borrow(cs).borrow_mut().delay_ms(300u16));
        led2.set_low();
        free(|cs| delay.borrow(cs).borrow_mut().delay_ms(200u16));
        break;
    }

    rprintln!("Creating Sensor Interface");

    // TODO PCB:
    let sck =
        gpioa
            .pa5
            .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    // TODO PCB:
    let mosi =
        gpioa
            .pa7
            .into_alternate_push_pull(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);
    let miso = gpioa
        .pa6
        .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrl);

    let mut cs = gpioa
        .pa8
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper); // CS pin
    cs.set_high(); // idle high
    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        MODE_0,
        1.MHz(),
        clocks,
        &mut apb2,
    );

    let mut ms5849_spi = MS5849::new_spi(spi, cs, &delay);

    rprintln!("Starting main waiting loop");
    loop {
        let pressure = ms5849_spi.measure_pressure_to_bar();
        match ms5849_spi.depth_or_altitude() {
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
        ms5849_spi.read_spi();
        match ms5849_spi.depth_or_altitude() {
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

fn millis_tim2() -> u32 {
    Timer::<TIM2>::count()
}

fn millis_tim2_since(start: u32) -> u32 {
    Timer::<TIM2>::count().wrapping_sub(start)
}
