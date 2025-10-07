#![no_main]
#![no_std]

use panic_probe as _;

use core::cell::RefCell;

use cortex_m::interrupt::{Mutex, free};
use cortex_m_rt::entry;

use stm32l4xx_hal::{
    delay::Delay,
    i2c::{Config, I2c},
    pac,
    prelude::*,
};

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
    let mut gpioa = dp.GPIOA.split(&mut ahb2);
    let mut gpiob = dp.GPIOB.split(&mut ahb2);
    let mut gpioc = dp.GPIOC.split(&mut ahb2);

    // Configure PA5 as push-pull output (needs moder + otyper blocks)
    let mut led = gpioa
        .pa5
        .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
    let mut led1 = gpiob
        .pb6
        .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
    let mut led2 = gpioc
        .pc7
        .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);

    // Delay using systick
    let delay: Mutex<RefCell<Delay>> = Mutex::new(RefCell::new(Delay::new(cp.SYST, clocks)));

    rprintln!("Creating Sensor Interface");

    let _communication_select_a =
        gpioa.pa8.into_push_pull_output_in_state(&mut gpioa.moder, &mut gpioa.otyper, PinState::High);
    let _communication_select_b =
        gpioa.pa10.into_push_pull_output_in_state(&mut gpioa.moder, &mut gpioa.otyper, PinState::High);
    // TODO PCB: pb10
    let scl =
        gpiob
            .pb8
            .into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    // scl.internal_pull_up(&mut gpiob.pupdr, true);
    // TODO PCB: pb11
    let sda =
        gpiob
            .pb9
            .into_alternate_open_drain(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
    // sda.internal_pull_up(&mut gpiob.pupdr, true);

    let config = Config::new(100_u32.kHz(), clocks.clone());
    // TODO PCB: I2C2
    let i2c_interface = I2c::i2c1(dp.I2C1, (scl, sda), config, &mut rcc.apb1r1);
    let ms5849_i2c = MS5849::new(i2c_interface, &delay);

    rprintln!("Starting main loop");

    loop {
        led.toggle();
        led1.toggle();
        led2.set_high();
        free(|cs| delay.borrow(cs).borrow_mut().delay_ms(300u16));
        led2.set_low();
        free(|cs| delay.borrow(cs).borrow_mut().delay_ms(200u16));

        let pressure = ms5849_i2c.measure_pressure_to_bar();
        match ms5849_i2c.depth() {
            Ok(Some(depth)) => rprintln!("Measuring Pressure: {:?}, depth: {:?}", pressure, depth),
            Ok(None) => rprintln!("No pressure or depth."),
            Err(e) => rprintln!(
                "Got error while reading depth for pressure {:?}: {:?}.",
                pressure,
                e
            ),
        };
    }
}
