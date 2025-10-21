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

use stm32l4xx_hal::{
    delay::Delay, hal::spi::MODE_0, interrupt, pac, prelude::*, spi::Spi, timer::Timer,
};

use rtt_target::{rprintln, rtt_init_print};

use thalmann::{
    display_utils::{format_f32, show_duration},
    dive::{DiveMeasurement, DiveProfile, StopSchedule},
    gas::GasMix,
    pressure_unit::{Bar, Pressure, msw},
};

use stdc_stm32_rs::{
    MS5849,
    display::{DisplayState, MAX_STOP_NUMS},
};

static TIM2_COUNTER: Mutex<RefCell<u32>> = Mutex::new(RefCell::new(0));
static DISPLAY_STATE: Mutex<RefCell<DisplayState>> =
    Mutex::new(RefCell::new(DisplayState::default()));

#[cortex_m_rt::interrupt]
fn TIM2() {
    free(|cs| {
        let mut counter = TIM2_COUNTER.borrow(cs).borrow_mut();
        *counter = counter.wrapping_add(1);
    });

    let dp = unsafe { pac::Peripherals::steal() };
    dp.TIM2.sr.modify(|_, w| w.uif().clear_bit()); // clear update interrupt flag
}

#[entry]
fn main() -> ! {
    rtt_init_print!();

    rprintln!("Booting..");

    let cp = cortex_m::Peripherals::take().unwrap();

    let dp = pac::Peripherals::take().unwrap();

    // Enable required power and flash interfaces
    let mut rcc = dp.RCC.constrain();
    let mut flash = dp.FLASH.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    // TODO: Configure clock to 24 MHz if required with .sysclk(24.MHz())
    let clocks = rcc.cfgr.freeze(&mut flash.acr, &mut pwr);

    let mut ahb2 = rcc.ahb2;
    let mut apb2 = rcc.apb2;
    let mut gpioa = dp.GPIOA.split(&mut ahb2);
    let mut gpiob = dp.GPIOB.split(&mut ahb2);
    let mut gpioc = dp.GPIOC.split(&mut ahb2);

    let mut timer = Timer::tim2(dp.TIM2, 1.kHz(), clocks, &mut rcc.apb1r1);
    timer.listen(stm32l4xx_hal::timer::Event::TimeOut); // enable interrupt on update
    unsafe { cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2) };

    // Configure PA5 as push-pull output (needs moder + otyper blocks)
    // let mut led = gpioa
    //     .pa5
    //     .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let mut led1 = gpiob
        .pb6
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut led2 = gpioc
        .pc7
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);

    // Delay using systick
    let delay: Mutex<RefCell<Delay>> = Mutex::new(RefCell::new(Delay::new(cp.SYST, clocks)));

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

    let gases: [GasMix<f32>; 1] = [thalmann::gas::AIR];

    let start_time = millis();
    rprintln!("Starting main loop");

    loop {
        let pressure = ms5849_spi.measure_pressure_to_bar();
        match (ms5849_spi.depth(), ms5849_spi.altitude()) {
            (Ok(Some(depth)), _) => {
                rprintln!(
                    "Starting Dive: Pressure: {:?}, depth: {:?}",
                    pressure,
                    depth
                );
                break;
            }
            (Err(_), Ok(Some(altitude))) => rprintln!(
                "Measuring Pressure: {:?}, altitude: {:?}",
                pressure,
                altitude
            ),
            (Ok(None), _) | (_, Ok(None)) => rprintln!("No pressure or depth."),
            (Err(e), Err(e2)) => rprintln!(
                "Got error while reading depth for pressure {:?}: {:?}, altitude: {:?}.",
                pressure,
                e,
                e2
            ),
        };

        // TODO: Remove once testing complete
        break;
    }

    let mut dive_profile = DiveProfile {
        dive_id: 1,
        max_depth: msw::new(0.0),
        gases: gases,
        measurements: [DiveMeasurement {
            depth: msw::new(0.0).to_pa(),
            time_ms: millis() as usize,
            gas: 0,
        }],
    };

    loop {
        let millis = millis();
        let duration_since_start = Duration::from_millis((millis - start_time) as u64);
        set_dive_time(duration_since_start);
        // led.toggle();
        led1.toggle();
        led2.set_high();
        free(|cs| delay.borrow(cs).borrow_mut().delay_ms(300u16));
        led2.set_low();
        free(|cs| delay.borrow(cs).borrow_mut().delay_ms(200u16));

        ms5849_spi.read_spi();
        let pressure = ms5849_spi.measure_pressure_to_bar();
        match (ms5849_spi.depth(), ms5849_spi.altitude()) {
            (Ok(Some(depth)), _) => {
                rprintln!("Measuring Pressure: {:?}, depth: {:?}", pressure, depth);
                set_depth(depth);
                // TODO: Add Measurement to Dive Profile
            }
            (Err(_), Ok(Some(altitude))) => rprintln!(
                "Measuring Pressure: {:?}, altitude: {:?}",
                pressure,
                altitude
            ),
            (Ok(None), _) | (_, Ok(None)) => rprintln!("No pressure or depth."),
            (Err(e), Err(e2)) => rprintln!(
                "Got error while reading depth for pressure {:?}: {:?}, altitude: {:?}.",
                pressure,
                e,
                e2
            ),
        };

        free(|cs| {
            let display_state = DISPLAY_STATE.borrow(cs).borrow();
            refresh_display(display_state);
        });
    }
}

fn set_depth(depth: Bar) {
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

fn refresh_display(display_state: Ref<DisplayState>) {
    let _duration_chars = show_duration(display_state.dive_time);
    let _meters_chars = format_f32::<' ', 3, 1>(display_state.depth.to_f32());
}

// Function to get current millis
fn millis() -> u32 {
    free(|cs| *TIM2_COUNTER.borrow(cs).borrow())
}
