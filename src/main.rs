#![no_main]
#![no_std]

use panic_probe as _;

use core::cell::RefCell;

use cortex_m::interrupt::{Mutex, free};
use cortex_m_rt::entry;

use stm32l4xx_hal::{delay::Delay, hal::spi::MODE_0, pac, prelude::*, spi::Spi};

use rtt_target::{rprintln, rtt_init_print};

use stdc_stm32_rs::MS5849;

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

    rprintln!("Starting main loop");

    loop {
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
                rprintln!("Measuring Pressure: {:?}, depth: {:?}", pressure, depth)
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
    }
}
