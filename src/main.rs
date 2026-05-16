#![no_main]
#![no_std]
#![feature(const_trait_impl)]
#![feature(const_default)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::{
    cell::RefCell,
    task::{Context, Poll},
};

use cortex_m::interrupt::{Mutex as CmMutex, free};
#[cfg(feature = "online_benchmarking")]
use cortex_m::register::primask;

use cortex_m_rt::exception;
use modes::{AppMode, SurfaceModeExit};
use rtic_monotonics::stm32::prelude::*;
use rtt_target::{rprintln, rtt_init_print};

#[cfg(not(feature = "online_benchmarking"))]
use stm32l4xx_hal::hal::timer::{Cancel, CountDown};
use stm32l4xx_hal::{
    hal::spi::MODE_0,
    i2c::{self, I2c},
    pac::{self, TIM5},
    prelude::*,
    pwr::Pwr,
    rcc::{APB1R1, Clocks, PllConfig, PllDivider},
    rtc::{Rtc, RtcClockSource, RtcConfig},
    serial::Serial,
    spi::Spi,
    timer::Timer,
};

use stdc_stm32_rs::{
    benchmarking,
    components::{
        MS5849,
        battery_status::{BatteryStatusI2C, Max17262Variant},
        flash::{Flash, SpiFlash},
        uart_log::{ExternalLogger, UartLogger},
    },
    concat_any_bytes,
    stm32::{Mono, noop_waker},
};

mod modes;
mod tasks;
mod types;

use stdc_diving_algorithms::{
    gas::{self, GasMix},
    pressure_unit::Pressure,
};
pub use types::*;

const ENABLE_BLUETOOTH: bool = true;
static SERIAL_NUMBER: [u8; 4] = [0, 0, 0, 0];
static BLUETOOTH_NAME: [u8; 8] = concat_any_bytes!(b"STDC", SERIAL_NUMBER);

const INITIAL_FLASH_ADDRESS: u32 = 1 << 21;
const BATTERY_UPDATE_INTERVAL_MILLIS: u32 = 30_000;
const BLUETOOTH_TASK_DELAY_MILLIS: u64 = 200;

#[cfg(not(feature = "online_benchmarking"))]
const STOP2_SURFACE_SLEEP_SECONDS: u32 = 2;

const GASES: [GasMix<f32>; 1] = [gas::AIR];

fn flash_log_bytes(bytes: &[u8]) {
    rprintln!("{}", str::from_utf8(bytes).unwrap());
}
fn sensor_log_bytes(_bytes: &[u8]) {}

#[cfg(not(feature = "online_benchmarking"))]
fn log_tim5_state(_: &str) {}

#[cfg(feature = "online_benchmarking")]
fn log_tim5_state(tag: &str) {
    let tim5 = unsafe { &*pac::TIM5::ptr() };
    let rcc = unsafe { &*pac::RCC::ptr() };
    let primask_active = primask::read().is_active();
    let tim5_enabled = cortex_m::peripheral::NVIC::is_enabled(pac::Interrupt::TIM5);
    let tim5_pending = cortex_m::peripheral::NVIC::is_pending(pac::Interrupt::TIM5);
    let exti0_enabled = cortex_m::peripheral::NVIC::is_enabled(pac::Interrupt::EXTI0);
    let exti0_pending = cortex_m::peripheral::NVIC::is_pending(pac::Interrupt::EXTI0);

    rprintln!(
        "{} TIM5: PRIMASK={} NVIC[TIM5 en={} pend={}] NVIC[EXTI0 en={} pend={}] RCC_APB1ENR1=0x{:08X} CR1=0x{:08X} DIER=0x{:08X} SR=0x{:08X} CNT=0x{:08X} CCR1=0x{:08X} CCR2=0x{:08X} PSC=0x{:08X} ARR=0x{:08X}",
        tag,
        primask_active,
        tim5_enabled,
        tim5_pending,
        exti0_enabled,
        exti0_pending,
        rcc.apb1enr1.read().bits(),
        tim5.cr1.read().bits(),
        tim5.dier.read().bits(),
        tim5.sr.read().bits(),
        tim5.cnt.read().bits(),
        tim5.ccr1.read().bits(),
        tim5.ccr2.read().bits(),
        tim5.psc.read().bits(),
        tim5.arr.read().bits(),
    );
}

static POWER_CUT_RED_INDICATOR: CmMutex<RefCell<Option<Pc9Output>>> =
    CmMutex::new(RefCell::new(None));

#[rtic::app(device = stm32l4xx_hal::pac, peripherals = true, dispatchers = [EXTI0])]
mod app {
    use rtic_monotonics::fugit::Duration;
    use stdc_diving_algorithms::{pressure_unit::Pa, setup::NUM_TISSUES};
    use stdc_stm32_rs::{
        components::display::{LedDisplay, SpiDisplay},
        stm32::TIM2_MONO_CLOCK,
    };

    use super::*;

    #[shared]
    struct Shared {
        latest_measurements: LatestMeasurements,
    }

    #[local]
    struct Local {
        mode: AppMode,
        dive_mode_indicator: Pc8Output,
        mode_surface_pressure: Pa,
        surface_mode_state: modes::surface::SurfaceModeState,
        bluetooth_mode_state: modes::bluetooth::BluetoothModeState,
        rtc: Rtc,
        display: DisplayDevice,
        flash: FlashDevice,
        ms5849_i2c: SensorMs5849,
        battery_status: BatteryGauge,
        logger: &'static LoggerMutex,
        clocks: stm32l4xx_hal::rcc::Clocks,
        apb1r1: stm32l4xx_hal::rcc::APB1R1,
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
        let dp = cx.device;

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

        // TODO: LSE crystal on first soldered PCB not responding (gets stuck after "RTC Config")
        //  so using LSI for now
        let rtc_config = RtcConfig::default().clock_config(RtcClockSource::LSI);
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

        rprintln!("RTC set up");

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
        let logger_obj = UartLogger::new(debug_tx, debug_rx);
        let logger: &'static LoggerMutex =
            cortex_m::singleton!(: LoggerMutex = CmMutex::new(RefCell::new(logger_obj))).unwrap();
        log_str(logger, "Logger to UART set up");

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

        log_str(logger, "Additional Pins for Display set up");

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
        let mut display = SpiDisplay::new(display_en, display_spi, spi_reset, not_data_command);
        // match display.turn_on() {
        //     Ok(_) => log_bytes(&logger, b"Turned on the display."),
        //     Err(e) => {
        //         log_bytes(&logger, b"Failed to turn on the display");
        //         log_bytes(&logger, e.details.as_bytes());
        //     }
        // }
        // match display.show_splashscreen(&BLUETOOTH_NAME) {
        //     Ok(_) => log_bytes(&logger, b"Showing Splash Screen"),
        //     Err(e) => {
        //         log_bytes(&logger, b"Failed to show the Splash Screen");
        //         log_bytes(&logger, e.details.as_bytes());
        //     }
        // }
        log_str(logger, "Display finished setting up");

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
        let mut flash: FlashDevice =
            SpiFlash::new(0, 1 << 22, flash_spi, flash_cs_nss, flash_log_bytes);

        log_str(logger, "Flash Device set up");

        let cur_addr = flash
            .read::<4>(INITIAL_FLASH_ADDRESS)
            .map(u32::from_be_bytes);
        log_str(logger, "Flash initial address read complete");
        match cur_addr {
            Err(_) | Ok(0) => {
                let new_pos = INITIAL_FLASH_ADDRESS + 4;
                modes::power_cut_mark_unsafe(modes::POWER_CUT_UNSAFE_FLASH_WRITE);
                let _ = flash.write(&new_pos.to_be_bytes());
                let _ = flash.set_pos(new_pos);
                modes::power_cut_mark_safe(modes::POWER_CUT_UNSAFE_FLASH_WRITE);
            }
            Ok(_) => {
                let cur_addr = cur_addr.unwrap();
                rprintln!("Current Flash Address: {}", cur_addr);
                rprintln!("Check: {}", cur_addr - (1 << 21));
                let _ = flash.set_pos(cur_addr);
            }
        }
        modes::power_cut_mark_unsafe(modes::POWER_CUT_UNSAFE_FLASH_WRITE);
        sync_power_cut_indicator();
        let _ = flash.write(&SERIAL_NUMBER);
        modes::power_cut_mark_safe(modes::POWER_CUT_UNSAFE_FLASH_WRITE);
        sync_power_cut_indicator();

        log_str(logger, "Flash write complete");

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
        let ms5849_i2c: SensorMs5849 = create_sensor_ms5849(sensor_i2c, sensor_log_bytes);

        log_str(logger, "Pressure Sensor set up");

        let start_surface_pressure = match ms5849_i2c.current_pressure_pa() {
            Some(p) => {
                let temp = ms5849_i2c.temperature();
                log_str(logger, "Got first result from Pressure Sensor");
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

        log_str(logger, "Battery Status set up");

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

        let bluetooth_mode_state: modes::bluetooth::BluetoothModeState =
            modes::bluetooth::BluetoothModeState::new();

        log_str(logger, "Finished init task, returning");

        (
            Shared {
                latest_measurements: LatestMeasurements::new(),
            },
            Local {
                mode: AppMode::Surface,
                dive_mode_indicator,
                mode_surface_pressure: start_surface_pressure,
                surface_mode_state: modes::surface::SurfaceModeState::new(),
                bluetooth_mode_state,
                rtc,
                display,
                flash,
                ms5849_i2c,
                battery_status,
                logger,
                clocks,
                apb1r1,
                bluetooth_usart3: Some(dp.USART3),
                bluetooth_hw_resources: Some((usart3_tx, usart3_rx, bluetooth_tx_ind)),
                bluetooth: None,
                bluetooth_initialized: false,
                dive_runtime: None,
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
            benchmarking::record_wfi();
            #[cfg(not(feature = "online_benchmarking"))]
            cortex_m::asm::wfi();

            #[cfg(feature = "online_benchmarking")]
            cortex_m::asm::nop();

            i += 1;
            if i % 100_000 == 0 {
                rprintln!("Idle for {}00K cycles", i / 100_000);
                log_tim5_state("Idle heartbeat");
            }
        }
    }

    enum TaskModeTickResult {
        EnterStop2,
        DirectContinue,
        Delay(Duration<u64, 1, { TIM2_MONO_CLOCK }>),
    }

    #[task(priority = 1, shared = [latest_measurements], local = [
        mode,
        dive_mode_indicator,
        mode_surface_pressure,
        surface_mode_state,
        bluetooth_mode_state,
        ms5849_i2c,
        display,
        flash,
        logger,
        rtc,
        clocks,
        apb1r1,
        bluetooth_usart3,
        bluetooth_hw_resources,
        bluetooth,
        bluetooth_initialized,
        dive_runtime,
        latest_calculations_state,
    ])]
    async fn task_mode_tick(mut cx: task_mode_tick::Context) {
        loop {
            modes::power_cut_mark_unsafe(modes::POWER_CUT_UNSAFE_TASK_RUNNING);
            sync_power_cut_indicator();
            sync_dive_mode_indicator(cx.local.dive_mode_indicator, cx.local.mode);
            rprintln!("Task Mode Tick in mode: {:?}", cx.local.mode);

            let result: TaskModeTickResult = match *cx.local.mode {
                AppMode::Surface => {
                    log_tim5_state("Before surface tick");
                    let (surface_exit, sample) =
                        benchmarking::measure_async("task.surface.tick", async {
                            log_tim5_state("Surface tick closure start");
                            let mut latest_measurements =
                                cx.shared.latest_measurements.lock(|m| *m);
                            let surface_exit = modes::surface::run_surface_mode_tick(
                                cx.local.surface_mode_state,
                                cx.local.ms5849_i2c,
                                cx.local.flash,
                                &mut latest_measurements,
                            )
                            .await;
                            log_tim5_state("Surface tick closure resumed");
                            cx.shared
                                .latest_measurements
                                .lock(|m| *m = latest_measurements);
                            surface_exit
                        })
                        .await;
                    log_tim5_state("After surface tick");
                    rprintln!("Log benchmark sample {:?}", sample);
                    benchmarking::log_sample(&sample);
                    rprintln!("Finished Surface Tick with exit mode {:?}", surface_exit);

                    match surface_exit {
                        Some(SurfaceModeExit::Dive(surface_pressure)) => {
                            *cx.local.mode_surface_pressure = surface_pressure;
                            *cx.local.mode = AppMode::Dive;
                            *cx.local.dive_runtime = Some(modes::dive::setup_dive_mode(
                                cx.local.flash,
                                cx.local.rtc,
                                cx.local.logger,
                                surface_pressure,
                                &GASES,
                            ));
                            TaskModeTickResult::DirectContinue
                        }
                        Some(SurfaceModeExit::Bluetooth(surface_pressure)) => {
                            *cx.local.mode_surface_pressure = surface_pressure;
                            *cx.local.mode = AppMode::Bluetooth;
                            cx.local.bluetooth_mode_state.on_enter();
                            TaskModeTickResult::DirectContinue
                        }
                        None => {
                            sync_dive_mode_indicator(cx.local.dive_mode_indicator, cx.local.mode);
                            TaskModeTickResult::EnterStop2
                        }
                    }
                }
                AppMode::Bluetooth => {
                    let bluetooth_ready = modes::bluetooth::ensure_bluetooth_initialized(
                        cx.local.clocks,
                        cx.local.apb1r1,
                        cx.local.bluetooth_usart3,
                        cx.local.bluetooth_hw_resources,
                        cx.local.bluetooth,
                        cx.local.bluetooth_initialized,
                        cx.local.logger,
                    );

                    if !bluetooth_ready
                        || modes::bluetooth::run_bluetooth_mode_tick(
                            cx.local.bluetooth_mode_state,
                            cx.local.bluetooth.as_mut().unwrap(),
                            cx.local.flash,
                            cx.local.logger,
                        )
                    {
                        transition_into_surface(cx.local.mode, cx.local.surface_mode_state);
                        TaskModeTickResult::DirectContinue
                    } else {
                        // TODO: Bluetooth Functionality
                        TaskModeTickResult::Delay(BLUETOOTH_TASK_DELAY_MILLIS.millis())
                    }
                }
                AppMode::Dive => {
                    let Some(runtime) = cx.local.dive_runtime.as_mut() else {
                        transition_into_surface(cx.local.mode, cx.local.surface_mode_state);
                        sync_dive_mode_indicator(cx.local.dive_mode_indicator, cx.local.mode);
                        let _ = task_mode_tick::spawn();
                        return;
                    };

                    let dive_exit = {
                        let mut latest_measurements = cx.shared.latest_measurements.lock(|m| *m);
                        let (dive_exit, sample) =
                            benchmarking::measure_async("task.dive.tick", async {
                                modes::dive::run_dive_mode_tick(
                                    runtime,
                                    cx.local.display,
                                    cx.local.ms5849_i2c,
                                    cx.local.flash,
                                    &mut latest_measurements,
                                    &mut cx.local.latest_calculations_state,
                                    cx.local.logger,
                                )
                                .await
                            })
                            .await;
                        cx.shared
                            .latest_measurements
                            .lock(|m| *m = latest_measurements);
                        benchmarking::log_sample(&sample);
                        dive_exit
                    };

                    if let Some(surface_pressure) = dive_exit {
                        *cx.local.mode_surface_pressure = surface_pressure;
                        *cx.local.dive_runtime = None;
                        transition_into_surface(cx.local.mode, cx.local.surface_mode_state);
                        TaskModeTickResult::DirectContinue
                    } else {
                        TaskModeTickResult::Delay(50_u64.millis())
                    }
                }
            };
            rprintln!("Completed Task Mode Tick");
            modes::power_cut_mark_safe(modes::POWER_CUT_UNSAFE_TASK_RUNNING);
            Mono::delay(500u64.millis()).await;
            match result {
                TaskModeTickResult::DirectContinue => {}
                TaskModeTickResult::Delay(duration) => {
                    Mono::delay(duration).await;
                }
                // TODO: Reenable STOP2 once it works with delay
                TaskModeTickResult::EnterStop2 => {
                    rprintln!("Going to sleep");
                    #[cfg(not(feature = "online_benchmarking"))]
                    {
                        enter_stop2_for_surface(cx.local.rtc);
                        let _ = wake_reinit::spawn();
                        // Allow scheduling of other tasks to avoid starving with this loop
                        Mono::delay(1u64.micros()).await;
                    }
                    #[cfg(feature = "online_benchmarking")]
                    Mono::delay(1000u64.millis()).await;
                }
            }
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

    #[task(priority = 1)]
    async fn wake_reinit(_: wake_reinit::Context) {
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
        rprintln!("Checking Delay");
        Mono::delay(2_u64.micros()).await;
        rprintln!("Spawning main task");
        let _ = task_mode_tick::spawn();
    }
}

fn init_clocks(
    cfgr: stm32l4xx_hal::rcc::CFGR,
    acr: &mut stm32l4xx_hal::flash::ACR,
    pwr: &mut Pwr,
    tim5: TIM5,
    apb1r1: &mut APB1R1,
) -> Clocks {
    let clocks = cfgr
        .sysclk_with_pll(8.MHz(), PllConfig::new(2, 8, PllDivider::Div8))
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
fn create_sensor_ms5849<L: Fn(&[u8]) -> ()>(i2c: SensorI2c, log_bytes: L) -> SensorMs5849 where {
    let mut fut = MS5849::new_i2c(i2c, log_bytes);
    let mut fut = unsafe { core::pin::Pin::new_unchecked(&mut fut) };
    let waker = noop_waker();
    let mut cx = Context::from_waker(&waker);
    loop {
        match fut.as_mut().poll(&mut cx) {
            Poll::Ready(val) => break val,
            Poll::Pending => {}
        };
        // TODO: Insert Delay or so here
    }
}

fn create_battery_status(battery_gauge_i2c: BatteryI2c) -> BatteryGauge {
    let mut fut = BatteryStatusI2C::new(battery_gauge_i2c, Max17262Variant::R);
    let mut fut = unsafe { core::pin::Pin::new_unchecked(&mut fut) };
    let waker = noop_waker();
    let mut cx = Context::from_waker(&waker);
    loop {
        match fut.as_mut().poll(&mut cx) {
            Poll::Ready(val) => break val,
            Poll::Pending => {}
        };
        // TODO: Insert Delay or so here
    }
}

fn transition_into_surface(
    mode: &mut AppMode,
    surface_mode_state: &mut modes::surface::SurfaceModeState,
) {
    *mode = AppMode::Surface;
    surface_mode_state.reset_for_entry();
}

fn log_str<L: ExternalLogger>(logger: &CmMutex<RefCell<L>>, str: &str) {
    rprintln!("{}", str);
    free(|cs| {
        let logged = logger.borrow(cs).borrow_mut().log_bytes(str.as_bytes());
        if logged.is_err() {
            rprintln!("Failed logging bytes to UART");
        }
    });
}

#[cfg(not(feature = "online_benchmarking"))]
fn enter_stop2_for_surface(rtc: &mut Rtc) {
    let mut wakeup_timer = rtc.wakeup_timer();
    wakeup_timer.start(STOP2_SURFACE_SLEEP_SECONDS);

    unsafe {
        (*pac::PWR::ptr()).scr.write(|w| {
            w.wuf1().set_bit();
            w.wuf2().set_bit();
            w.wuf3().set_bit();
            w.wuf4().set_bit();
            w.wuf5().set_bit();
            w
        });

        (*pac::PWR::ptr()).cr1.modify(|_, w| w.lpms().bits(0b010));
    }

    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    cp.SCB.set_sleepdeep();
    cortex_m::asm::dsb();
    cortex_m::asm::wfi();
    cp.SCB.clear_sleepdeep();

    let _ = wakeup_timer.cancel();
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

fn sync_dive_mode_indicator(dive_mode_indicator: &mut Pc8Output, mode: &AppMode) {
    if matches!(mode, AppMode::Dive) {
        let _ = dive_mode_indicator.set_low();
    } else {
        let _ = dive_mode_indicator.set_high();
    }
}

#[exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    rprintln!("HARDFAULT: {:?}", ef);
    loop {}
}

// Copied from panic_probe
// Panic handler for `probe-run`.
//
// When this panic handler is used, panics will make `probe-run` print a backtrace and exit with a
// non-zero status code, indicating failure. This building block can be used to run on-device
// tests.
//

use core::panic::PanicInfo;
use core::sync::atomic::{AtomicBool, Ordering};

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    static PANICKED: AtomicBool = AtomicBool::new(false);
    rprintln!("PANIC");

    cortex_m::interrupt::disable();

    // Guard against infinite recursion, just in case.
    if !PANICKED.load(Ordering::Relaxed) {
        PANICKED.store(true, Ordering::Relaxed);

        rprintln!("{}", info);
    }

    hard_fault();
}

/// Trigger a `HardFault` via `udf` instruction.
#[cfg(target_os = "none")]
pub fn hard_fault() -> ! {
    // If `UsageFault` is enabled, we disable that first, since otherwise `udf` will cause that
    // exception instead of `HardFault`.
    {
        const SHCSR: *mut u32 = 0xE000ED24usize as _;
        const USGFAULTENA: usize = 18;

        unsafe {
            let mut shcsr = core::ptr::read_volatile(SHCSR);
            shcsr &= !(1 << USGFAULTENA);
            core::ptr::write_volatile(SHCSR, shcsr);
        }
    }

    cortex_m::asm::udf();
}
