#![no_main]
#![no_std]

use panic_probe as _;
use rtt_target::{rprintln, rtt_init_print};

use stm32l4xx_hal::{delay::Delay, prelude::*, serial::Serial};

use stdc_stm32_rs::components::uart_log::{ExternalLogger, UartLogger};

type Pc5Rail = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Output<stm32l4xx_hal::gpio::PushPull>,
    stm32l4xx_hal::gpio::L8,
    'C',
    5,
>;
type Pc8Led = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Output<stm32l4xx_hal::gpio::OpenDrain>,
    stm32l4xx_hal::gpio::H8,
    'C',
    8,
>;
type Pc9Led = stm32l4xx_hal::gpio::Pin<
    stm32l4xx_hal::gpio::Output<stm32l4xx_hal::gpio::OpenDrain>,
    stm32l4xx_hal::gpio::H8,
    'C',
    9,
>;
type DebugTx = stm32l4xx_hal::serial::Tx<stm32l4xx_hal::pac::USART1>;
type DebugRx = stm32l4xx_hal::serial::Rx<stm32l4xx_hal::pac::USART1>;
type DebugLogger = UartLogger<DebugTx, DebugRx>;

fn log_line<L: ExternalLogger>(logger: &mut L, msg: &str) {
    rprintln!("{}", msg);
    if logger.log_bytes(msg.as_bytes()).is_err() || logger.log_bytes(b"\r\n").is_err() {
        rprintln!("UART log write failed");
    }
}

#[rtic::app(device = stm32l4xx_hal::pac, peripherals = true)]
mod app {
    use stm32l4xx_hal::rcc::{PllConfig, PllDivider};

    use super::*;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        delay: Delay,
        logger: DebugLogger,
        display_power_rail: Pc5Rail,
        dive_led: Pc8Led,
        red_led: Pc9Led,
        outputs_on: bool,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        rtt_init_print!();
        rprintln!("Booting minimal hardware verification firmware");

        let mut rcc = cx.device.RCC.constrain();
        let mut flash = cx.device.FLASH.constrain();
        let mut pwr = cx.device.PWR.constrain(&mut rcc.apb1r1);
        let mut ahb2 = rcc.ahb2;
        let mut apb2 = rcc.apb2;
        let mut gpioa = cx.device.GPIOA.split(&mut ahb2);
        let mut gpiob = cx.device.GPIOB.split(&mut ahb2);
        let mut gpioc = cx.device.GPIOC.split(&mut ahb2);

        let clocks = rcc
            .cfgr
            .sysclk_with_pll(32.MHz(), PllConfig::new(1, 8, PllDivider::Div4))
            .pclk1(32.MHz())
            .pclk2(32.MHz())
            .freeze(&mut flash.acr, &mut pwr);

        let usart1_tx =
            gpioa
                .pa9
                .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
        let usart1_rx =
            gpioa
                .pa10
                .into_alternate(&mut gpioa.moder, &mut gpioa.otyper, &mut gpioa.afrh);
        let debug_uart = Serial::usart1(
            cx.device.USART1,
            (usart1_tx, usart1_rx),
            115_200.bps(),
            clocks,
            &mut apb2,
        );
        let (debug_tx, debug_rx) = debug_uart.split();
        let mut logger = UartLogger::new(debug_tx, debug_rx);
        log_line(&mut logger, "UART logger ready");

        let mut display_cs = gpioa
            .pa4
            .into_push_pull_output(&mut gpioa.moder, &mut gpioa.otyper);
        let mut display_not_data_command = gpiob
            .pb0
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        let mut bluetooth_tx_ind = gpiob
            .pb2
            .into_pull_down_input(&mut gpiob.moder, &mut gpiob.pupdr);
        let mut flash_cs = gpiob
            .pb12
            .into_push_pull_output(&mut gpiob.moder, &mut gpiob.otyper);
        let mut display_reset = gpioc
            .pc4
            .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);

        let _ = display_cs.set_high();
        let _ = display_not_data_command.set_low();
        let _ = flash_cs.set_high();
        let _ = display_reset.set_low();

        let _ = &mut bluetooth_tx_ind;

        let mut display_power_rail = gpioc
            .pc5
            .into_push_pull_output(&mut gpioc.moder, &mut gpioc.otyper);
        let mut dive_led = gpioc
            .pc8
            .into_open_drain_output(&mut gpioc.moder, &mut gpioc.otyper);
        let mut red_led = gpioc
            .pc9
            .into_open_drain_output(&mut gpioc.moder, &mut gpioc.otyper);

        let _ = display_power_rail.set_low();
        let _ = dive_led.set_high();
        let _ = red_led.set_high();
        log_line(&mut logger, "Safety states configured; rail off");

        (
            Shared {},
            Local {
                delay: Delay::new(cx.core.SYST, clocks),
                logger,
                display_power_rail,
                dive_led,
                red_led,
                outputs_on: false,
            },
        )
    }

    #[idle(local = [delay, logger, display_power_rail, dive_led, red_led, outputs_on])]
    fn idle(cx: idle::Context) -> ! {
        loop {
            let _ = cx.local.display_power_rail.set_low();

            if *cx.local.outputs_on {
                let _ = cx.local.dive_led.set_high();
                let _ = cx.local.red_led.set_high();
                log_line(cx.local.logger, "LEDs OFF, rail OFF");
            } else {
                let _ = cx.local.dive_led.set_low();
                let _ = cx.local.red_led.set_low();
                log_line(cx.local.logger, "LEDs ON, rail OFF");
            }

            *cx.local.outputs_on = !*cx.local.outputs_on;
            cx.local.delay.delay_ms(300_u16);
        }
    }
}
