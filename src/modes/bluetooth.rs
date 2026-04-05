use core::cell::RefCell;

use cortex_m::interrupt::{Mutex, free};
use rtt_target::rprintln;
use stm32l4xx_hal::{pac, prelude::*, serial::Serial};

use stdc_stm32_rs::components::uart_log::ExternalLogger;

use super::{millis_tim2, millis_tim2_since};

const BLUETOOTH_MODE_INACTIVITY_TIMEOUT_MILLIS: u32 = 20_000;

pub struct BluetoothModeState {
    last_activity_millis: u32,
    logged_entry: bool,
    logged_timeout: bool,
}

impl BluetoothModeState {
    pub fn new() -> Self {
        Self {
            last_activity_millis: 0,
            logged_entry: false,
            logged_timeout: false,
        }
    }

    pub fn on_enter(&mut self) {
        self.last_activity_millis = millis_tim2();
        self.logged_entry = false;
        self.logged_timeout = false;
    }

    pub fn mark_activity(&mut self) {
        self.last_activity_millis = millis_tim2();
        self.logged_timeout = false;
    }
}

pub fn run_bluetooth_mode_tick<L: ExternalLogger>(
    state: &mut BluetoothModeState,
    logger: &Mutex<RefCell<L>>,
) -> bool {
    if !state.logged_entry {
        log_bytes(logger, b"Bluetooth mode entered (placeholder)");
        rprintln!("Bluetooth mode entered (placeholder)");
        state.logged_entry = true;
    }

    let inactive_timed_out =
        millis_tim2_since(state.last_activity_millis) >= BLUETOOTH_MODE_INACTIVITY_TIMEOUT_MILLIS;

    // TODO: Detect connection, handle commands

    if inactive_timed_out && !state.logged_timeout {
        log_bytes(logger, b"Bluetooth mode exiting due to inactivity timeout");
        rprintln!("Bluetooth mode exiting due to inactivity timeout");
        state.logged_timeout = true;
    }

    inactive_timed_out
}

pub fn ensure_bluetooth_initialized<L: ExternalLogger>(
    clocks: &stm32l4xx_hal::rcc::Clocks,
    apb1r1: &mut stm32l4xx_hal::rcc::APB1R1,
    bluetooth_usart3: &mut Option<pac::USART3>,
    bluetooth_hw_resources: &mut Option<(
        crate::Pb10Usart3Tx,
        crate::Pb11Usart3Rx,
        crate::Pb2InputPullDown,
    )>,
    bluetooth: &mut Option<crate::BluetoothModule>,
    bluetooth_initialized: &mut bool,
    logger: &Mutex<RefCell<L>>,
) -> bool {
    if !crate::ENABLE_BLUETOOTH {
        return true;
    }

    if *bluetooth_initialized {
        return bluetooth.is_some();
    }

    if bluetooth.is_none() {
        let (Some(usart3), Some((usart3_tx, usart3_rx, tx_ind))) =
            (bluetooth_usart3.take(), bluetooth_hw_resources.take())
        else {
            log_bytes(logger, b"Bluetooth resources unavailable for lazy init");
            return false;
        };

        let bluetooth_uart = Serial::usart3(
            usart3,
            (usart3_tx, usart3_rx),
            115_200.bps(),
            *clocks,
            apb1r1,
        );
        let (bluetooth_tx, bluetooth_rx) = bluetooth_uart.split();
        *bluetooth = Some(crate::BluetoothModule {
            uart_tx: bluetooth_tx,
            uart_rx: bluetooth_rx,
            tx_ind,
        });
    }

    let Some(bluetooth_module) = bluetooth.take() else {
        return false;
    };

    match bluetooth_module.initialize_bluetooth(&crate::BLUETOOTH_NAME) {
        Ok(initialized_module) => {
            *bluetooth = Some(initialized_module);
            *bluetooth_initialized = true;
            true
        }
        Err(_) => {
            log_bytes(logger, b"Bluetooth lazy init failed");
            false
        }
    }
}

fn log_bytes<L: ExternalLogger>(logger: &Mutex<RefCell<L>>, bytes: &[u8]) {
    free(|cs| {
        if let Err(_) = logger.borrow(cs).borrow_mut().log_bytes(bytes) {
            rprintln!("Failed logging bytes to UART");
        }
    });
}
