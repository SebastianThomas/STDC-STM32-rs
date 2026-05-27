#![no_main]
#![no_std]
#![feature(const_trait_impl)]
#![feature(const_default)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![feature(stmt_expr_attributes)]

use core::cell::RefCell;
use core::sync::atomic::AtomicBool;
#[cfg(feature = "bluetooth")]
use core::sync::atomic::Ordering;

use cortex_m::interrupt::{Mutex as CmMutex, free};

use modes::{AppMode, SurfaceModeExit};
use rtic_monotonics::{fugit::Duration, stm32::prelude::*};
use rtt_target::{rprintln, rtt_init_print};

#[cfg(feature = "bluetooth")]
use stm32l4xx_hal::gpio::Edge;
use stm32l4xx_hal::gpio::ExtiPin;
#[cfg(not(feature = "online_benchmarking"))]
use stm32l4xx_hal::hal::timer::Cancel;
use stm32l4xx_hal::{
    hal::spi::MODE_0,
    i2c::{self, I2c},
    pac::{self},
    prelude::*,
    rtc::{Rtc, RtcClockSource, RtcConfig, RtcWakeupClockSource},
    serial::Serial,
    spi::Spi,
};

#[cfg(feature = "live_sim")]
use stdc_stm32_rs::components::LiveSimEmulationControl;
use stdc_stm32_rs::{
    benchmarking,
    components::{
        battery_status::{BatteryStatusI2C, Max17262Variant},
        display::*,
        flash::{Flash, SpiFlash},
    },
    concat_any_bytes,
    stm32::{Mono, TIM2_MONO_CLOCK, noop_waker},
};

mod modes;
mod tasks;
mod types;

use stdc_diving_algorithms::{
    gas::{self, GasMix},
    pressure_unit::{Pa, Pressure},
    setup::NUM_TISSUES,
};
pub use types::*;

#[cfg(feature = "bluetooth")]
use modes::bluetooth::BluetoothModeState;

#[cfg(not(feature = "bluetooth"))]
#[derive(Clone, Copy, Debug)]
struct BluetoothModeState;

#[cfg(not(feature = "bluetooth"))]
impl BluetoothModeState {
    const fn new() -> Self {
        Self
    }

    #[allow(unused)]
    fn on_enter(&mut self) {}

    #[allow(unused)]
    fn on_exit(&mut self) {}
}

mod runtime;
pub use runtime::board::{
    create_battery_status, create_display, create_flash_device, create_pressure_sensor,
    init_clocks, sync_dive_mode_indicator, sync_power_cut_indicator, transition_into_surface,
};

#[cfg(not(feature = "online_benchmarking"))]
pub use runtime::board::{enter_stop2_for_surface, reinit_after_stop2};

static SERIAL_NUMBER: [u8; 4] = [0, 0, 0, 0];
static BLUETOOTH_NAME: [u8; 8] = concat_any_bytes!(b"STDC", SERIAL_NUMBER);

const FLASH_LOG_POINTER_ADDRESS: u32 = 1 << 21;
const FLASH_LOG_DATA_START_ADDRESS: u32 = FLASH_LOG_POINTER_ADDRESS + 4;
const FLASH_CAPACITY_BYTES: u32 = 1 << 22;
const BATTERY_UPDATE_INTERVAL_MILLIS: u32 = 30_000;
#[allow(unused)]
const BLUETOOTH_TASK_DELAY_MILLIS: u64 = 200;

#[cfg(not(feature = "online_benchmarking"))]
const STOP2_SURFACE_SLEEP_SECONDS: u32 = 2;

const NR_GASES: usize = 4;
const GASES: [GasMix<f32>; NR_GASES] = [gas::AIR, gas::NX100, gas::NX50, gas::TMX10_80];
const GASES_ENABLED: [bool; NR_GASES] = [true; 4];

static POWER_CUT_RED_INDICATOR: CmMutex<RefCell<Option<Pc9Output>>> =
    CmMutex::new(RefCell::new(None));
static BLUETOOTH_MODE_ACTIVE: AtomicBool = AtomicBool::new(false);

pub(crate) fn persist_flash_log_position(
    flash: &mut FlashDevice,
    next_pos: u32,
) -> Result<(), stdc_stm32_rs::components::spi_utils::SpiError> {
    if !(FLASH_LOG_DATA_START_ADDRESS..=FLASH_CAPACITY_BYTES).contains(&next_pos) {
        return Err(stdc_stm32_rs::components::spi_utils::SpiError::new(
            0,
            "Flash log position out of range",
        ));
    }

    flash.set_pos(FLASH_LOG_POINTER_ADDRESS)?;
    flash.write(&next_pos.to_be_bytes())?;
    flash.set_pos(next_pos)?;
    Ok(())
}

#[rtic::app(device = stm32l4xx_hal::pac, peripherals = true, dispatchers = [EXTI0])]
mod app {
    use stm32l4xx_hal::{pac::EXTI, rtc::Event};

    use super::*;

    #[shared]
    struct Shared {
        latest_measurements: LatestMeasurements,
        clocks: stm32l4xx_hal::rcc::Clocks,
        apb1r1: stm32l4xx_hal::rcc::APB1R1,
        flash: FlashDevice,
    }

    #[local]
    struct Local {
        mode: AppMode,
        dive_mode_indicator: Pc8Output,
        bluetooth_button: Pa1InputPullUp,
        mode_surface_pressure: Pa,
        surface_mode_state: modes::surface::SurfaceModeState,
        rtc: Rtc,
        display: DisplayDevice,
        ms5849_i2c: SensorMs5849,
        battery_status: BatteryGauge,
        bluetooth_mode_state: BluetoothModeState,
        bluetooth_usart3: Option<pac::USART3>,
        bluetooth_hw_resources: Option<(Pb10Usart3Tx, Pb11Usart3Rx, Pb2InputPullDown)>,
        bluetooth: Option<BluetoothModule>,
        bluetooth_initialized: bool,
        dive_runtime: Option<modes::dive::DiveRuntime<{ GASES.len() }>>,
        latest_calculations_state: LatestCalculationsState<{ NUM_TISSUES }, Pa>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        rtt_init_print!();
        rprintln!("Booting..");

        let mut core = cx.core;
        #[cfg(not(feature = "bluetooth"))]
        let dp = cx.device;
        #[cfg(feature = "bluetooth")]
        let mut dp = cx.device;

        benchmarking::enable_cycle_counter(&mut core);

        // Ensure RTT over DMA to fix wfi reattach issue
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

        let mut apb1r1 = rcc.apb1r1;
        let mut ahb2 = rcc.ahb2;
        let mut apb2 = rcc.apb2;
        let mut gpioa = dp.GPIOA.split(&mut ahb2);
        let mut gpiob = dp.GPIOB.split(&mut ahb2);
        let mut gpioc = dp.GPIOC.split(&mut ahb2);

        let clocks = init_clocks(rcc.cfgr, &mut flash_pac.acr, &mut pwr, dp.TIM5, &mut apb1r1);

        rprintln!("Clocks, TIM5, Delay set up");

        // TODO: LSE crystal on first soldered PCB not responding
        //  so using LSI for now
        let rtc_config = RtcConfig::default()
            .clock_config(RtcClockSource::LSI)
            .wakeup_clock_config(RtcWakeupClockSource::CkSpre);
        rprintln!("RTC Config");
        let mut rtc: Rtc = Rtc::rtc(dp.RTC, &mut apb1r1, &mut rcc.bdcr, &mut pwr.cr1, rtc_config);
        rprintln!("RTC Config II");
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

        let exti = unsafe { &mut *(pac::EXTI::ptr() as *mut EXTI) };
        rtc.listen(exti, Event::WakeupTimer);
        let is_interrupt = rtc.check_interrupt(Event::WakeupTimer, true);
        rprintln!("Was interrupt before: {}", is_interrupt);
        unsafe {
            cortex_m::peripheral::NVIC::unmask(pac::Interrupt::RTC_WKUP);
        }

        rprintln!("RTC & WakeupTimer set up");

        // PA13: JTMS-SWDIO
        let _tms =
            gpioa
                .pa13
                .into_alternate::<0>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
        // PA14: JTCK-SWCLK
        let _tck =
            gpioa
                .pa14
                .into_alternate::<0>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
        // PA15: JTDI
        let _tdi =
            gpioa
                .pa15
                .into_alternate::<0>(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
        // PB3: JTDO-TRACESWO
        let _tdo =
            gpiob
                .pb3
                .into_alternate::<0>(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrl);

        rprintln!("SWD set up");

        let mut dive_mode_indicator = gpioc
            .pc8
            .into_open_drain_output(&mut gpioc.moder, &mut gpioc.otyper);
        let red_indicator = gpioc
            .pc9
            .into_open_drain_output(&mut gpioc.moder, &mut gpioc.otyper);
        free(|cs| {
            *POWER_CUT_RED_INDICATOR.borrow(cs).borrow_mut() = Some(red_indicator);
        });

        let usart1_tx =
            gpioa
                .pa9
                .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
        let usart1_rx =
            gpioa
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
        rprintln!("Logger to UART still available; will use rprintln and UART once.");
        {
            use stdc_stm32_rs::components::uart_log::{ExternalLogger, UartLogger};
            let mut uart_logger = UartLogger::new(debug_tx, debug_rx);
            if let Err(e) = uart_logger.log_bytes(b"Initialized UART\r\n") {
                rprintln!("UART logger write failed: {:?}", e);
            }
        }

        // TODO: Check reset is actually push pull output for Display
        let spi_reset = gpioc
            .pc4
            .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
        let display_en = gpioc
            .pc5
            .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
        let not_data_command = gpiob
            .pb0
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);

        rprintln!("Additional Pins for Display set up");

        let mut cs = gpioa
            .pa4
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        let sck = gpioa.pa5.into_alternate_push_pull(
            &mut gpioa.moder,
            &mut gpioa.otyper,
            &mut gpioa.afrl,
        );
        let mosi = gpioa.pa7.into_alternate_push_pull(
            &mut gpioa.moder,
            &mut gpioa.otyper,
            &mut gpioa.afrl,
        );
        // MISO Only for type checker - not connected on PCB, no input possible
        let miso_nc = gpioa.pa6.into_alternate_push_pull(
            &mut gpioa.moder,
            &mut gpioa.otyper,
            &mut gpioa.afrl,
        );
        cs.set_high();
        let display_spi = Spi::spi1(
            dp.SPI1,
            (sck, miso_nc, mosi),
            MODE_0,
            1.MHz(),
            clocks,
            &mut apb2,
        );
        let display = create_display(display_en, display_spi, spi_reset, not_data_command);
        rprintln!("Display finished setting up");

        let mut flash_cs_nss = gpiob
            .pb12
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        let sck = gpiob.pb13.into_alternate_push_pull(
            &mut gpiob.moder,
            &mut gpiob.otyper,
            &mut gpiob.afrh,
        );
        let miso = gpiob.pb14.into_alternate_push_pull(
            &mut gpiob.moder,
            &mut gpiob.otyper,
            &mut gpiob.afrh,
        );
        let mosi = gpiob.pb15.into_alternate_push_pull(
            &mut gpiob.moder,
            &mut gpiob.otyper,
            &mut gpiob.afrh,
        );
        flash_cs_nss.set_high();
        let flash_spi = Spi::spi2(
            dp.SPI2,
            (sck, miso, mosi),
            MODE_0,
            1.MHz(),
            clocks,
            &mut apb1r1,
        );
        let mut flash: FlashDevice = create_flash_device(flash_spi, flash_cs_nss);

        rprintln!("Flash Device set up");

        let cur_addr = flash
            .read::<4>(FLASH_LOG_POINTER_ADDRESS)
            .map(u32::from_be_bytes);
        rprintln!("Flash initial address read complete: {:?}", cur_addr);
        match cur_addr {
            Ok(pos) if (FLASH_LOG_DATA_START_ADDRESS..=FLASH_CAPACITY_BYTES).contains(&pos) => {
                rprintln!("Current Flash Address: {}", pos);
                rprintln!("Check: {}", pos - FLASH_LOG_POINTER_ADDRESS);
                let _ = flash.set_pos(pos);
            }
            Ok(pos) => {
                let new_pos = FLASH_LOG_DATA_START_ADDRESS;
                rprintln!("Initial flash address invalid, resetting to {:?}", new_pos);
                rprintln!("Invalid persisted position was {:?}", pos);
                let _ = modes::flash_write_and_measure("flash.init.reset_pos", || {
                    persist_flash_log_position(&mut flash, new_pos)
                });
            }
            Err(e) => {
                let new_pos = FLASH_LOG_DATA_START_ADDRESS;
                rprintln!("Initial flash address invalid, resetting to {:?}", new_pos);
                rprintln!("Reading persisted position failed: {:?}", e);
                let _ = modes::flash_write_and_measure("flash.init.reset_pos", || {
                    persist_flash_log_position(&mut flash, new_pos)
                });
            }
        }
        sync_power_cut_indicator();
        let serial_write_start =
            modes::flash_write_and_measure("flash.serial.write", || flash.write(&SERIAL_NUMBER));
        if let Ok(start) = serial_write_start {
            let next_pos = start + SERIAL_NUMBER.len() as u32;
            let _ = modes::flash_write_and_measure("flash.serial.persist_pos", || {
                persist_flash_log_position(&mut flash, next_pos)
            });
        }
        sync_power_cut_indicator();

        rprintln!("Flash write complete");

        let mut pressure_sensor_enable_pin = gpiob
            .pb8
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        let _ = pressure_sensor_enable_pin.set_high();
        let scl = gpiob.pb6.into_alternate_open_drain(
            &mut gpiob.moder,
            &mut gpiob.otyper,
            &mut gpiob.afrl,
        );
        let sda = gpiob.pb7.into_alternate_open_drain(
            &mut gpiob.moder,
            &mut gpiob.otyper,
            &mut gpiob.afrl,
        );
        let sensor_i2c = I2c::i2c1(
            dp.I2C1,
            (scl, sda),
            i2c::Config::new(100.kHz(), clocks),
            &mut apb1r1,
        );
        #[cfg(feature = "live_sim")]
        let mut ms5849_i2c: SensorMs5849 = create_pressure_sensor(sensor_i2c);
        #[cfg(not(feature = "live_sim"))]
        let ms5849_i2c: SensorMs5849 = create_pressure_sensor(sensor_i2c);

        rprintln!("Pressure Sensor set up");

        let start_surface_pressure = match ms5849_i2c.current_pressure_pa() {
            Some(p) => {
                let temp = ms5849_i2c.temperature();
                rprintln!("Got first result from Pressure Sensor");
                rprintln!(
                    "Start Pressure: {} Pa, Temperature: {}",
                    p.to_f32(),
                    temp.unwrap(),
                );
                p
            }
            None => {
                rprintln!("Got error while measuring pressure");
                DEFAULT_SURFACE_PRESSURE
            }
        };
        #[cfg(feature = "live_sim")]
        ms5849_i2c.enter_live_sim(start_surface_pressure);

        let mut scl = gpioc.pc0.into_alternate_open_drain(
            &mut gpioc.moder,
            &mut gpioc.otyper,
            &mut gpioc.afrl,
        );
        // TODO: Disable internal Pull-up once Pull-ups are added on PCB
        scl.internal_pull_up(&mut gpioc.pupdr, true);
        let mut sda = gpioc.pc1.into_alternate_open_drain(
            &mut gpioc.moder,
            &mut gpioc.otyper,
            &mut gpioc.afrl,
        );
        // TODO: Disable internal Pull-up once Pull-ups are added on PCB
        sda.internal_pull_up(&mut gpioc.pupdr, true);
        let battery_gauge_i2c = I2c::i2c3(
            dp.I2C3,
            (scl, sda),
            i2c::Config::new(100.kHz(), clocks),
            &mut apb1r1,
        );
        let battery_status: BatteryGauge = create_battery_status(battery_gauge_i2c);

        rprintln!("Battery Status set up");

        #[cfg(not(feature = "bluetooth"))]
        let bluetooth_button = gpioa
            .pa1
            .into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr);
        #[cfg(feature = "bluetooth")]
        let mut bluetooth_button = gpioa
            .pa1
            .into_pull_up_input(&mut gpioa.moder, &mut gpioa.pupdr);
        #[cfg(feature = "bluetooth")]
        {
            bluetooth_button.make_interrupt_source(&mut dp.SYSCFG, &mut apb2);
            bluetooth_button.enable_interrupt(&mut dp.EXTI);
            bluetooth_button.trigger_on_edge(&mut dp.EXTI, Edge::Falling);
            unsafe {
                cortex_m::peripheral::NVIC::unmask(pac::Interrupt::EXTI1);
            }
        }
        let bluetooth_tx_ind = gpiob
            .pb2
            .into_pull_down_input(&mut gpiob.moder, &mut gpiob.pupdr);
        let usart3_tx =
            gpiob
                .pb10
                .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);
        let usart3_rx =
            gpiob
                .pb11
                .into_alternate(&mut gpiob.moder, &mut gpiob.otyper, &mut gpiob.afrh);

        // Active-low indicator: start with LED off until Dive mode is entered.
        let _ = dive_mode_indicator.set_high();

        modes::power_cut_mark_safe(modes::POWER_CUT_UNSAFE_TASK_RUNNING);

        let _ = task_mode_tick::spawn();
        let _ = task_battery_update::spawn();

        #[cfg(feature = "bluetooth")]
        runtime::bluetooth::request_bluetooth_mode(&BLUETOOTH_MODE_ACTIVE, || {
            task_bluetooth_mode::spawn()
        });

        rprintln!("Finished init task, returning");

        #[cfg(feature = "live_sim")]
        let initial_mode = AppMode::Dive;
        #[cfg(not(feature = "live_sim"))]
        let initial_mode = AppMode::Surface;

        #[cfg(feature = "live_sim")]
        let initial_dive_runtime = Some(modes::dive::setup_dive_mode(
            &mut flash,
            &rtc,
            start_surface_pressure,
            &GASES,
            &GASES_ENABLED,
        ));
        #[cfg(not(feature = "live_sim"))]
        let initial_dive_runtime = None;

        (
            Shared {
                latest_measurements: LatestMeasurements::new(),
                clocks,
                apb1r1,
                flash,
            },
            Local {
                mode: initial_mode,
                dive_mode_indicator,
                mode_surface_pressure: start_surface_pressure,
                surface_mode_state: modes::surface::SurfaceModeState::new(),
                bluetooth_button,
                bluetooth_mode_state: BluetoothModeState::new(),
                bluetooth_usart3: Some(dp.USART3),
                bluetooth_hw_resources: Some((usart3_tx, usart3_rx, bluetooth_tx_ind)),
                bluetooth: None,
                bluetooth_initialized: false,
                rtc,
                display,
                ms5849_i2c,
                battery_status,
                dive_runtime: initial_dive_runtime,
                latest_calculations_state: LatestCalculationsState::new(
                    start_surface_pressure,
                    None,
                ),
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        rprintln!("First idle execution");
        let mut i: u64 = 0;
        loop {
            #[cfg(not(feature = "online_benchmarking"))]
            cortex_m::asm::wfi();

            #[cfg(feature = "online_benchmarking")]
            cortex_m::asm::nop();

            i += 1;
            if i % 100_000 == 0 {
                rprintln!("Idle for {}00K cycles", i / 100_000);
            }
        }
    }

    #[task(binds = EXTI1, priority = 2, local = [bluetooth_button])]
    fn task_bluetooth_button(cx: task_bluetooth_button::Context) {
        if cx.local.bluetooth_button.check_interrupt() {
            cx.local.bluetooth_button.clear_interrupt_pending_bit();
            runtime::bluetooth::request_bluetooth_mode(&BLUETOOTH_MODE_ACTIVE, || {
                task_bluetooth_mode::spawn()
            });
        }
    }

    #[task(priority = 1, shared = [clocks, apb1r1, flash], local = [
        bluetooth_mode_state,
        bluetooth_usart3,
        bluetooth_hw_resources,
        bluetooth,
        bluetooth_initialized,
    ])]
    #[allow(unused)]
    async fn task_bluetooth_mode(mut cx: task_bluetooth_mode::Context) {
        #[cfg(feature = "bluetooth")]
        {
            // Bluetooth mode task - spawned by PA1 button interrupt
            cx.local.bluetooth_mode_state.on_enter();
            let mut bluetooth_command_mode_guard_done = false;

            let mut timeouts: u32 = 0;

            loop {
                if !BLUETOOTH_MODE_ACTIVE.load(Ordering::Acquire) {
                    break;
                }

                if !bluetooth_command_mode_guard_done {
                    Mono::delay(120u64.millis()).await;
                    bluetooth_command_mode_guard_done = true;
                }

                // Try to ensure Bluetooth hardware is initialized using shared clocks/apb1r1
                let bluetooth_ready = cx.shared.clocks.lock(|clocks| {
                    cx.shared.apb1r1.lock(|apb1r1| {
                        modes::bluetooth::ensure_bluetooth_initialized(
                            clocks,
                            apb1r1,
                            &mut cx.local.bluetooth_usart3,
                            &mut cx.local.bluetooth_hw_resources,
                            &mut cx.local.bluetooth,
                            &mut cx.local.bluetooth_initialized,
                        )
                    })
                });

                if !bluetooth_ready {
                    Mono::delay(BLUETOOTH_TASK_DELAY_MILLIS.millis()).await;
                    continue;
                }

                if let Some(bluetooth) = cx.local.bluetooth.as_mut() {
                    cx.shared.flash.lock(|flash| {
                        let timeout = modes::bluetooth::run_bluetooth_mode_tick(
                            &mut cx.local.bluetooth_mode_state,
                            bluetooth,
                            flash,
                        );
                        if timeout {
                            rprintln!("Bluetooth Timeout #{}", timeouts);
                            timeouts += 1;
                            if timeouts >= 20 {
                                BLUETOOTH_MODE_ACTIVE.store(false, Ordering::Release);
                            }
                        } else {
                            timeouts = 0;
                        }
                    });
                }
                rprintln!("Bluetooth Tick Delay");
                Mono::delay(BLUETOOTH_TASK_DELAY_MILLIS.millis()).await;
                rprintln!("Bluetooth Tick After Delay");
            }

            BLUETOOTH_MODE_ACTIVE.store(false, Ordering::Release);
        }

        #[cfg(not(feature = "bluetooth"))]
        {
            let _ = cx;
        }
    }

    #[task(binds = RTC_WKUP, priority = 2)]
    fn task_rtc_wakeup(_: task_rtc_wakeup::Context) {
        let rtc = unsafe { &*pac::RTC::ptr() };
        rtc.isr.modify(|_, w| w.wutf().clear_bit());
        unsafe {
            (*pac::EXTI::ptr()).pr1.write(|w| w.bits(1 << 20));
        }
        rprintln!("RTC wakeup interrupt fired");
    }

    #[allow(unused)]
    enum TaskModeTickResult {
        EnterStop2,
        DirectContinue,
        Delay(Duration<u64, 1, { TIM2_MONO_CLOCK }>),
        DelayUntil(Duration<u64, 1, { TIM2_MONO_CLOCK }>),
        #[cfg(feature = "live_sim")]
        HaltMCU(bool),
    }

    #[task(priority = 1, shared = [latest_measurements, clocks, apb1r1, flash], local = [
        mode,
        dive_mode_indicator,
        mode_surface_pressure,
        surface_mode_state,
        ms5849_i2c,
        display,
        rtc,
        dive_runtime,
        latest_calculations_state,
    ])]
    async fn task_mode_tick(mut cx: task_mode_tick::Context) {
        loop {
            let iteration_start = Mono::now();

            modes::power_cut_mark_unsafe(modes::POWER_CUT_UNSAFE_TASK_RUNNING);
            sync_power_cut_indicator();
            sync_dive_mode_indicator(cx.local.dive_mode_indicator, cx.local.mode);
            rprintln!("Task Mode Tick in mode: {:?}", cx.local.mode);

            let result = benchmarking::measure_async_and_log("task.mode.tick.loop", async {
                match *cx.local.mode {
                    AppMode::Surface => {
                        let (surface_exit, latest_measurements, surface_flash_log) =
                            benchmarking::measure_async_and_log("task.surface.tick", async {
                                let latest_measurements =
                                    cx.shared.latest_measurements.lock(|m| *m);
                                let (surface_exit, latest_measurements, surface_flash_log) =
                                    modes::surface::run_surface_mode_tick(
                                        cx.local.surface_mode_state,
                                        cx.local.ms5849_i2c,
                                        latest_measurements,
                                    )
                                    .await;
                                (surface_exit, latest_measurements, surface_flash_log)
                            })
                            .await;
                        rprintln!("Finished Surface Tick with exit mode {:?}", surface_exit);

                        cx.shared
                            .latest_measurements
                            .lock(|m| *m = latest_measurements);
                        if let Some((current_measurement_millis, pressure)) = surface_flash_log {
                            cx.shared.flash.lock(|flash| {
                                modes::surface::log_data_flash(
                                    current_measurement_millis,
                                    pressure,
                                    &latest_measurements,
                                    flash,
                                    cx.local.surface_mode_state,
                                );
                            });
                        }
                        match surface_exit {
                            Some(SurfaceModeExit::Dive(surface_pressure)) => {
                                *cx.local.mode_surface_pressure = surface_pressure;
                                *cx.local.mode = AppMode::Dive;
                                #[cfg(feature = "live_sim")]
                                cx.local.ms5849_i2c.enter_live_sim(surface_pressure);
                                *cx.local.dive_runtime = Some(cx.shared.flash.lock(|flash| {
                                    modes::dive::setup_dive_mode(
                                        flash,
                                        cx.local.rtc,
                                        surface_pressure,
                                        &GASES,
                                        &GASES_ENABLED,
                                    )
                                }));
                                TaskModeTickResult::DirectContinue
                            }
                            None => {
                                sync_dive_mode_indicator(
                                    cx.local.dive_mode_indicator,
                                    cx.local.mode,
                                );
                                TaskModeTickResult::EnterStop2
                            }
                        }
                    }
                    AppMode::Dive => {
                        #[cfg(feature = "bluetooth")]
                        if BLUETOOTH_MODE_ACTIVE.load(Ordering::Acquire) {
                            rprintln!("Deactivate Bluetooth");
                            BLUETOOTH_MODE_ACTIVE.store(false, Ordering::Release);
                        }

                        let Some(runtime) = cx.local.dive_runtime.as_mut() else {
                            transition_into_surface(cx.local.mode, cx.local.surface_mode_state);
                            sync_dive_mode_indicator(cx.local.dive_mode_indicator, cx.local.mode);
                            return TaskModeTickResult::DirectContinue;
                        };

                        let (dive_exit, latest_measurements, flash_log) = {
                            let latest_measurements = cx.shared.latest_measurements.lock(|m| *m);
                            let (dive_exit, latest_measurements, flash_log) =
                                modes::dive::run_dive_mode_tick(
                                    runtime,
                                    cx.local.display,
                                    cx.local.ms5849_i2c,
                                    latest_measurements,
                                    &mut cx.local.latest_calculations_state,
                                )
                                .await;
                            (dive_exit, latest_measurements, flash_log)
                        };

                        cx.shared
                            .latest_measurements
                            .lock(|m| *m = latest_measurements);
                        if let Some(flash_log) = flash_log {
                            cx.shared.flash.lock(|flash| {
                                modes::dive::write_flash_log(
                                    runtime,
                                    flash,
                                    flash_log,
                                    &latest_measurements,
                                );
                            });
                        }

                        if let Some(surface_pressure) = dive_exit {
                            #[cfg(feature = "live_sim")]
                            let is_benchmark_profile = runtime.is_benchmark_profile;

                            *cx.local.mode_surface_pressure = surface_pressure;
                            *cx.local.dive_runtime = None;

                            #[cfg(feature = "live_sim")]
                            cx.local.ms5849_i2c.exit_live_sim();

                            transition_into_surface(cx.local.mode, cx.local.surface_mode_state);
                            sync_dive_mode_indicator(cx.local.dive_mode_indicator, cx.local.mode);

                            #[cfg(feature = "live_sim")]
                            return TaskModeTickResult::HaltMCU(is_benchmark_profile);
                            #[cfg(not(feature = "live_sim"))]
                            return TaskModeTickResult::DirectContinue;
                        } else {
                            TaskModeTickResult::DelayUntil(200_u64.millis())
                        }
                    }
                }
            })
            .await;
            modes::power_cut_mark_safe(modes::POWER_CUT_UNSAFE_TASK_RUNNING);
            match result {
                TaskModeTickResult::DirectContinue => {}
                TaskModeTickResult::Delay(duration) => {
                    Mono::delay(duration).await;
                }
                TaskModeTickResult::DelayUntil(duration) => {
                    let target = iteration_start + duration;
                    Mono::delay_until(target).await;
                }
                TaskModeTickResult::EnterStop2 => {
                    rprintln!("Going to sleep");
                    #[cfg(not(feature = "online_benchmarking"))]
                    {
                        enter_stop2_for_surface(cx.local.rtc).await;
                        reinit_after_stop2();
                    }
                    #[cfg(feature = "online_benchmarking")]
                    Mono::delay(1000u64.millis()).await;
                }
                #[cfg(feature = "online_benchmarking")]
                TaskModeTickResult::HaltMCU(is_benchmark_profile) => {
                    if is_benchmark_profile {
                        rprintln!("Benchmark dive completed; stopping MCU");
                        crate::runtime::board::stop_mcu_after_benchmark();
                    }
                }
            }
            Mono::delay(1u64.millis()).await;
        }
    }

    #[task(priority = 1, shared = [latest_measurements], local = [battery_status])]
    async fn task_battery_update(mut cx: task_battery_update::Context) {
        loop {
            rprintln!("Task Battery Update Tick: Starting");

            cx.shared.latest_measurements.lock(|latest_measurements| {
                tasks::battery::calc_battery_status(cx.local.battery_status, latest_measurements);
            });
            rprintln!("Task Battery Update Tick: Finished");

            tasks::battery::wait_for_next_battery_update().await;
        }
    }
}
