use core::task::{Context, Poll};
use cortex_m::interrupt::free;

use stm32l4xx_hal::{
    hal::blocking::spi::{Transfer, Write},
    pac::TIM5,
    prelude::*,
    pwr::Pwr,
    rcc::{APB1R1, Clocks, PllConfig, PllDivider},
    spi::Spi,
    timer::Timer,
};

#[cfg(not(feature = "online_benchmarking"))]
use stm32l4xx_hal::rtc::Rtc;

use crate::*;

pub fn init_clocks(
    cfgr: stm32l4xx_hal::rcc::CFGR,
    acr: &mut stm32l4xx_hal::flash::ACR,
    pwr: &mut Pwr,
    tim5: TIM5,
    apb1r1: &mut APB1R1,
) -> Clocks {
    let clocks = cfgr
        .sysclk_with_pll(16.MHz(), PllConfig::new(2, 8, PllDivider::Div8))
        .freeze(acr, pwr);

    // Timer input clock = PCLK1 * 2 when APB1 prescaler > 1.
    let tim2_clk = if clocks.pclk1() == clocks.hclk() {
        clocks.pclk1()
    } else {
        clocks.pclk1() * 2
    };
    rprintln!("TIM2 input clock: {:?} MHz", tim2_clk.to_MHz());
    Mono::start(tim2_clk.to_Hz());

    rprintln!(
        "Clocks: Sysclk: {:?}, PCLK1: {:?}, PCLK2: {:?}, HCLK: {:?}, ",
        clocks.sysclk(),
        clocks.pclk1(),
        clocks.pclk2(),
        clocks.hclk()
    );

    Timer::free_running_tim5(tim5, clocks, 1.kHz(), false, apb1r1);

    clocks
}

pub fn create_sensor_ms5849(i2c: SensorI2c) -> SensorMs5849 {
    let mut fut = MS5849::new_i2c(i2c);
    let mut fut = unsafe { core::pin::Pin::new_unchecked(&mut fut) };
    let waker = noop_waker();
    let mut cx = Context::from_waker(&waker);
    loop {
        match fut.as_mut().poll(&mut cx) {
            Poll::Ready(val) => break val,
            Poll::Pending => {}
        };
    }
}

pub fn create_battery_status(battery_gauge_i2c: BatteryI2c) -> BatteryGauge {
    let mut fut = BatteryStatusI2C::new(battery_gauge_i2c, Max17262Variant::R);
    let mut fut = unsafe { core::pin::Pin::new_unchecked(&mut fut) };
    let waker = noop_waker();
    let mut cx = Context::from_waker(&waker);
    loop {
        match fut.as_mut().poll(&mut cx) {
            Poll::Ready(val) => break val,
            Poll::Pending => {}
        };
    }
}

pub fn create_flash_device<SPI: Transfer<u8> + Write<u8>, CS: OutputPin>(
    flash_spi: SPI,
    flash_cs_nss: CS,
) -> SpiFlash<SPI, CS> {
    SpiFlash::new(0, 1 << 22, flash_spi, flash_cs_nss)
}

pub fn create_pressure_sensor(i2c: SensorI2c) -> SensorMs5849 {
    create_sensor_ms5849(i2c)
}

#[cfg(not(feature = "display"))]
pub fn create_display<SPI, PINS, EN: OutputPin, RST: OutputPin, NDC: OutputPin>(
    display_en: EN,
    display_spi: Spi<SPI, PINS>,
    spi_reset: RST,
    not_data_command: NDC,
) -> SpiDisplay<SPI, PINS, EN, RST, NDC> {
    SpiDisplay::new(display_en, display_spi, spi_reset, not_data_command)
}

#[cfg(feature = "display")]
pub fn create_display<SPI, PINS, EN: OutputPin, RST: OutputPin, NDC: OutputPin>(
    display_en: EN,
    display_spi: Spi<SPI, PINS>,
    spi_reset: RST,
    not_data_command: NDC,
) -> SpiDisplay<SPI, PINS, EN, RST, NDC>
where
    Spi<SPI, PINS>: _embedded_hal_blocking_spi_Write<u8>,
{
    let mut display = SpiDisplay::new(display_en, display_spi, spi_reset, not_data_command);
    match display.turn_on() {
        Ok(_) => rprintln!("Turned on the display."),
        Err(e) => {
            rprintln!("Failed to turn on the display");
            rprintln!("{:?}", e.details.as_bytes());
        }
    }
    match display.show_splashscreen(&BLUETOOTH_NAME) {
        Ok(_) => rprintln!("Showing Splash Screen"),
        Err(e) => {
            rprintln!("Failed to show the Splash Screen");
            rprintln!("{:?}", e.details.as_bytes());
        }
    }
    display
}

pub fn transition_into_surface(
    mode: &mut AppMode,
    surface_mode_state: &mut modes::surface::SurfaceModeState,
) {
    *mode = AppMode::Surface;
    surface_mode_state.reset_for_entry();
}

#[cfg(not(feature = "online_benchmarking"))]
pub async fn enter_stop2_for_surface(rtc: &mut Rtc) {
    let mut wakeup_timer = rtc.wakeup_timer();
    wakeup_timer.start(STOP2_SURFACE_SLEEP_SECONDS);

    let pwr = unsafe { &*pac::PWR::ptr() };
    let mut cp = unsafe { cortex_m::Peripherals::steal() };

    pwr.scr.write(|w| {
        w.wuf1().set_bit();
        w.wuf2().set_bit();
        w.wuf3().set_bit();
        w.wuf4().set_bit();
        w.wuf5().set_bit();
        w
    });

    pwr.cr1.modify(|_, w| unsafe { w.lpms().bits(0b010) });

    rprintln!("Go into Deep Sleep");

    cp.SCB.set_sleepdeep();
    cortex_m::asm::dsb();
    cortex_m::asm::wfi();
    cp.SCB.clear_sleepdeep();

    let _ = wakeup_timer.cancel();

    rprintln!("Exit STOP2 for surface");
}

#[cfg(not(feature = "online_benchmarking"))]
pub fn reinit_after_stop2() {
    rprintln!("Waking up, reinit clocks");
    let dp = unsafe { pac::Peripherals::steal() };

    dp.RCC.ahb1enr.modify(|_, w| w.dma1en().set_bit());
    dp.DBGMCU.cr.modify(|_, w| {
        w.trace_ioen().set_bit();
        w.dbg_sleep().set_bit();
        w.dbg_stop().set_bit();
        w.dbg_standby().set_bit()
    });

    let mut rcc = dp.RCC.constrain();
    let mut flash_pac = dp.FLASH.constrain();
    let mut pwr = dp.PWR.constrain(&mut rcc.apb1r1);

    let _clocks = init_clocks(
        rcc.cfgr,
        &mut flash_pac.acr,
        &mut pwr,
        dp.TIM5,
        &mut rcc.apb1r1,
    );
}

pub fn sync_power_cut_indicator() {
    free(|cs| {
        if let Some(red_indicator) = POWER_CUT_RED_INDICATOR.borrow(cs).borrow_mut().as_mut() {
            if modes::is_power_cut_safe() {
                let _ = red_indicator.set_high();
            } else {
                let _ = red_indicator.set_low();
            }
        }
    });
}

pub fn sync_dive_mode_indicator(dive_mode_indicator: &mut Pc8Output, mode: &AppMode) {
    if matches!(mode, AppMode::Dive) {
        let _ = dive_mode_indicator.set_low();
    } else {
        let _ = dive_mode_indicator.set_high();
    }
}

#[cfg(feature = "online_benchmarking")]
pub fn stop_mcu_after_benchmark() -> ! {
    rprintln!("Benchmark complete, halting MCU");
    loop {
        cortex_m::asm::bkpt();
    }
}

#[cfg(not(feature = "online_benchmarking"))]
pub fn stop_mcu_after_benchmark() {
    rprintln!("Benchmark complete (no-op on non-benchmark build)");
}
