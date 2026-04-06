use core::cell::RefCell;
use core::fmt::Write;

use cortex_m::interrupt::{Mutex, free};
use heapless::String;
use rtt_target::rprintln;
use stm32l4xx_hal::{pac, prelude::*, serial::Serial};

use stdc_stm32_rs::components::{
    bluetooth::BluetoothModule as BluetoothIo,
    dive_log::{LogDiveControlDataBlock, LogPointData},
    flash::Flash,
    uart_log::ExternalLogger,
};

use super::{millis_tim2, millis_tim2_since};

const BLUETOOTH_MODE_INACTIVITY_TIMEOUT_MILLIS: u32 = 20_000;
const BLUETOOTH_MODE_MAX_SESSION_MILLIS: u32 = 60_000;
const MAX_RX_BYTES_PER_TICK: usize = 128;
const MAX_COMMAND_LINE_BYTES: usize = 640;
const MAX_FW_CHUNK_BYTES: usize = 128;
const MAX_LOG_CONTROL_GASES: usize = 16;
const MAX_LOG_CONTROL_BYTES: usize = 24 + MAX_LOG_CONTROL_GASES * 3;
const LOG_POINT_BASIC_BYTES: usize = 8;
const LOG_POINT_DECO_BYTES: usize = 8;
const MAX_LOG_POINT_BYTES: usize = LOG_POINT_BASIC_BYTES + LOG_POINT_DECO_BYTES;

const FW_SLOT_START_ADDRESS: u32 = 0x000000;
const LOG_POINTER_ADDRESS: u32 = 1 << 21;
const LOG_DATA_START_ADDRESS: u32 = LOG_POINTER_ADDRESS + 4;
const FLASH_CAPACITY_BYTES: u32 = 1 << 22;
const FW_SLOT_END_EXCLUSIVE: u32 = LOG_POINTER_ADDRESS;

struct FirmwareTransferState {
    active: bool,
    expected_size: u32,
    received: u32,
    next_addr: u32,
}

impl FirmwareTransferState {
    const fn new() -> Self {
        Self {
            active: false,
            expected_size: 0,
            received: 0,
            next_addr: FW_SLOT_START_ADDRESS,
        }
    }

    fn start(&mut self, size: u32) {
        self.active = true;
        self.expected_size = size;
        self.received = 0;
        self.next_addr = FW_SLOT_START_ADDRESS;
    }

    fn reset(&mut self) {
        self.active = false;
        self.expected_size = 0;
        self.received = 0;
        self.next_addr = FW_SLOT_START_ADDRESS;
    }
}

pub struct BluetoothModeState {
    entered_millis: u32,
    last_activity_millis: u32,
    logged_entry: bool,
    logged_timeout: bool,
    command_line_buf: [u8; MAX_COMMAND_LINE_BYTES],
    command_line_len: usize,
    fw_transfer: FirmwareTransferState,
    log_cursor: u32,
    log_end: u32,
}

impl BluetoothModeState {
    pub fn new() -> Self {
        Self {
            entered_millis: 0,
            last_activity_millis: 0,
            logged_entry: false,
            logged_timeout: false,
            command_line_buf: [0; MAX_COMMAND_LINE_BYTES],
            command_line_len: 0,
            fw_transfer: FirmwareTransferState::new(),
            log_cursor: LOG_DATA_START_ADDRESS,
            log_end: LOG_DATA_START_ADDRESS,
        }
    }

    pub fn on_enter(&mut self) {
        let now = millis_tim2();
        self.entered_millis = now;
        self.last_activity_millis = now;
        self.logged_entry = false;
        self.logged_timeout = false;
        self.command_line_len = 0;
        self.fw_transfer.reset();
        self.log_cursor = LOG_DATA_START_ADDRESS;
        self.log_end = LOG_DATA_START_ADDRESS;
    }

    pub fn mark_activity(&mut self) {
        self.last_activity_millis = millis_tim2();
        self.logged_timeout = false;
    }
}

pub fn run_bluetooth_mode_tick<L: ExternalLogger>(
    state: &mut BluetoothModeState,
    bluetooth: &mut crate::BluetoothModule,
    flash: &mut crate::FlashDevice,
    logger: &Mutex<RefCell<L>>,
) -> bool {
    if !state.logged_entry {
        log_bytes(logger, b"Bluetooth mode entered");
        rprintln!("Bluetooth mode entered");
        let _ = bluetooth.write_bytes(b"READY\r\n");
        state.logged_entry = true;
    }

    poll_and_handle_bluetooth_rx(state, bluetooth, flash, logger);

    let idle_elapsed = millis_tim2_since(state.last_activity_millis);
    let session_elapsed = millis_tim2_since(state.entered_millis);
    let inactive_timed_out = idle_elapsed >= BLUETOOTH_MODE_INACTIVITY_TIMEOUT_MILLIS;
    let session_timed_out = session_elapsed >= BLUETOOTH_MODE_MAX_SESSION_MILLIS;
    let timed_out = inactive_timed_out || session_timed_out;

    if timed_out && !state.logged_timeout {
        if session_timed_out {
            log_bytes(logger, b"Bluetooth mode exiting due to session timeout");
            rprintln!("Bluetooth mode exiting due to session timeout");
        } else {
            log_bytes(logger, b"Bluetooth mode exiting due to inactivity timeout");
            rprintln!("Bluetooth mode exiting due to inactivity timeout");
        }
        state.logged_timeout = true;
    }

    timed_out
}

fn poll_and_handle_bluetooth_rx<L: ExternalLogger>(
    state: &mut BluetoothModeState,
    bluetooth: &mut crate::BluetoothModule,
    flash: &mut crate::FlashDevice,
    logger: &Mutex<RefCell<L>>,
) {
    for _ in 0..MAX_RX_BYTES_PER_TICK {
        let byte = match bluetooth.try_read_byte() {
            Ok(Some(b)) => b,
            Ok(None) => return,
            Err(_) => {
                send_line(bluetooth, b"ERR UART");
                return;
            }
        };

        state.mark_activity();

        if byte == b'\n' {
            if state.command_line_len == 0 {
                continue;
            }

            let line_len = state.command_line_len;
            let mut line = [0u8; MAX_COMMAND_LINE_BYTES];
            line[..line_len].copy_from_slice(&state.command_line_buf[..line_len]);
            let line_bytes = &line[..line_len];
            handle_command_line(state, line_bytes, bluetooth, flash, logger);
            state.command_line_len = 0;
            continue;
        }

        if byte == b'\r' {
            continue;
        }

        if state.command_line_len >= state.command_line_buf.len() {
            state.command_line_len = 0;
            send_line(bluetooth, b"ERR LINE_TOO_LONG");
            continue;
        }

        state.command_line_buf[state.command_line_len] = byte;
        state.command_line_len += 1;
    }
}

fn handle_command_line<L: ExternalLogger>(
    state: &mut BluetoothModeState,
    line_bytes: &[u8],
    bluetooth: &mut crate::BluetoothModule,
    flash: &mut crate::FlashDevice,
    logger: &Mutex<RefCell<L>>,
) {
    let Ok(line) = core::str::from_utf8(line_bytes) else {
        send_line(bluetooth, b"ERR UTF8");
        return;
    };

    let mut parts = line.split_ascii_whitespace();
    let Some(command) = parts.next() else {
        send_line(bluetooth, b"ERR EMPTY");
        return;
    };

    match command {
        "PING" => {
            send_line(bluetooth, b"PONG");
        }
        "HELP" => {
            send_line(
                bluetooth,
                b"CMDS PING HELP LOG_BEGIN LOG_NEXT FW_BEGIN<size_hex> FW_CHUNK<payload_hex> FW_END FW_ABORT",
            );
        }
        "LOG_BEGIN" => match flash.read::<4>(LOG_POINTER_ADDRESS) {
            Ok(end_bytes) => {
                let mut end = u32::from_be_bytes(end_bytes);
                if end < LOG_DATA_START_ADDRESS || end > FLASH_CAPACITY_BYTES {
                    end = LOG_DATA_START_ADDRESS;
                }
                state.log_cursor = LOG_DATA_START_ADDRESS;
                state.log_end = end;

                if state.log_end <= LOG_DATA_START_ADDRESS {
                    send_line(bluetooth, b"LOG_EOF");
                    return;
                }

                match decode_control_block(flash, LOG_DATA_START_ADDRESS, state.log_end) {
                    Ok((control, control_len)) => {
                        let mut response: String<512> = String::new();
                        let _ = write!(
                            response,
                            "{{\"type\":\"control\",\"addr\":\"{:08X}\",\"len\":{},\"firmware_version\":{},\"computer_serial_nr\":{},\"log_model_version\":{},\"time_offset\":{},\"surface_interval\":{},\"dive_number\":{},\"surface_pressure_hpa\":{},\"surface_temperature_c\":{:.1},\"ascent_rate_agg_seconds\":{},\"salinity\":\"{:?}\",\"dive_mode\":\"{:?}\",\"deco_algorithm\":\"{:?}\",\"gf_low\":{},\"gf_high\":{},\"gas_nr\":{},\"gas_content_hex\":\"",
                            LOG_DATA_START_ADDRESS,
                            control_len,
                            control.firmware_version(),
                            control.computer_serial_number(),
                            control.log_model_version(),
                            control.time_offset(),
                            control.surface_interval(),
                            control.dive_number(),
                            control.surface_pressure_hpa(),
                            control.surface_temperature_half_c() as f32 / 2.0,
                            control.ascent_rate_agg_seconds(),
                            control.salinity_value(),
                            control.dive_mode_value(),
                            control.deco_algorithm_value(),
                            control.gf_low_value(),
                            control.gf_high_value(),
                            control.gas_nr,
                        );
                        append_hex(&mut response, control.gas_content_bytes());
                        let _ = write!(response, "\",\"raw_hex\":\"");
                        append_hex(&mut response, &control.raw_bytes());
                        let _ = write!(response, "\"}}");
                        send_line(bluetooth, response.as_bytes());

                        state.log_cursor = state.log_cursor.saturating_add(control_len as u32);
                    }
                    Err(_) => {
                        send_line(bluetooth, b"ERR LOG_CONTROL_DECODE");
                    }
                }
            }
            Err(_) => send_line(bluetooth, b"ERR FLASH_READ"),
        },
        "LOG_NEXT" => {
            if state.log_cursor >= state.log_end {
                send_line(bluetooth, b"LOG_EOF");
                return;
            }

            match decode_log_point(flash, state.log_cursor, state.log_end) {
                Ok((point, consumed)) => {
                    let mut response: String<512> = String::new();
                    let _ = write!(
                        response,
                        "{{\"type\":\"point\",\"addr\":\"{:08X}\",\"len\":{},\"time_delta_s\":{},\"depth_m\":{:.2},\"metadata\":{},\"dive_mode_active\":{},\"deco_obligation\":{},\"level_state\":\"{:?}\",\"optional_pages\":[{},{},{},{}],\"battery_percent\":{},\"temperature_c\":{:.1},\"ascent_rate\":{},\"deco_hex\":",
                        state.log_cursor,
                        consumed,
                        point.basic_data().time_delta_seconds(),
                        point.basic_data().depth().to_num::<f32>(),
                        point.basic_data().metadata().byte(),
                        point.basic_data().metadata().dive_mode_active(),
                        point.basic_data().metadata().deco_obligation(),
                        point.basic_data().metadata().level_state(),
                        point.basic_data().metadata().optional_param_pages()[0],
                        point.basic_data().metadata().optional_param_pages()[1],
                        point.basic_data().metadata().optional_param_pages()[2],
                        point.basic_data().metadata().optional_param_pages()[3],
                        point.basic_data().battery(),
                        point.basic_data().temperature() as f32 / 2.0,
                        point.basic_data().ascent_rate(),
                    );

                    if let Some(deco) = point.deco_bytes() {
                        let _ = write!(response, "\"");
                        append_hex(&mut response, &deco);
                        let _ = write!(response, "\"");
                    } else {
                        let _ = write!(response, "null");
                    }

                    let _ = write!(response, ",\"raw_hex\":\"");
                    append_hex(&mut response, &point.basic_bytes());
                    if let Some(deco) = point.deco_bytes() {
                        append_hex(&mut response, &deco);
                    }
                    let _ = write!(response, "\"}}");

                    send_line(bluetooth, response.as_bytes());
                    state.log_cursor = state.log_cursor.saturating_add(consumed as u32);
                }
                Err(_) => {
                    send_line(bluetooth, b"ERR LOG_POINT_DECODE");
                }
            }
        }
        "FW_BEGIN" => {
            let Some(size_hex) = parts.next() else {
                send_line(bluetooth, b"ERR FW_SIZE");
                return;
            };
            let Some(size) = parse_u32_hex(size_hex) else {
                send_line(bluetooth, b"ERR FW_SIZE_HEX");
                return;
            };
            if size == 0 || size > (FW_SLOT_END_EXCLUSIVE - FW_SLOT_START_ADDRESS) {
                send_line(bluetooth, b"ERR FW_SIZE_RANGE");
                return;
            }

            let erase_result = erase_flash_range_4k(
                flash,
                FW_SLOT_START_ADDRESS,
                FW_SLOT_START_ADDRESS + size,
            );
            if erase_result.is_err() {
                send_line(bluetooth, b"ERR FW_ERASE");
                log_bytes(logger, b"FW erase failed");
                return;
            }

            state.fw_transfer.start(size);
            send_line(bluetooth, b"OK FW_BEGIN");
        }
        "FW_CHUNK" => {
            if !state.fw_transfer.active {
                send_line(bluetooth, b"ERR FW_NOT_ACTIVE");
                return;
            }

            let Some(payload_hex) = parts.next() else {
                send_line(bluetooth, b"ERR FW_PAYLOAD");
                return;
            };

            let mut decoded = [0u8; MAX_FW_CHUNK_BYTES];
            let Some(decoded_len) = decode_hex(payload_hex.as_bytes(), &mut decoded) else {
                send_line(bluetooth, b"ERR FW_PAYLOAD_HEX");
                return;
            };

            if decoded_len == 0 {
                send_line(bluetooth, b"ERR FW_PAYLOAD_EMPTY");
                return;
            }

            if state.fw_transfer.received + decoded_len as u32 > state.fw_transfer.expected_size {
                send_line(bluetooth, b"ERR FW_OVERFLOW");
                return;
            }

            let write_res = write_flash_linear(
                flash,
                state.fw_transfer.next_addr,
                &decoded[..decoded_len],
            );
            if write_res.is_err() {
                send_line(bluetooth, b"ERR FW_WRITE");
                return;
            }

            state.fw_transfer.next_addr = state.fw_transfer.next_addr.saturating_add(decoded_len as u32);
            state.fw_transfer.received = state.fw_transfer.received.saturating_add(decoded_len as u32);

            let mut response: String<48> = String::new();
            let _ = write!(response, "OK {:08X}", state.fw_transfer.received);
            send_line(bluetooth, response.as_bytes());
        }
        "FW_END" => {
            if !state.fw_transfer.active {
                send_line(bluetooth, b"ERR FW_NOT_ACTIVE");
                return;
            }
            if state.fw_transfer.received != state.fw_transfer.expected_size {
                send_line(bluetooth, b"ERR FW_SIZE_MISMATCH");
                return;
            }
            state.fw_transfer.reset();
            send_line(bluetooth, b"OK FW_END");
        }
        "FW_ABORT" => {
            state.fw_transfer.reset();
            send_line(bluetooth, b"OK FW_ABORT");
        }
        _ => {
            send_line(bluetooth, b"ERR UNKNOWN_CMD");
        }
    }
}

fn send_line(bluetooth: &mut crate::BluetoothModule, msg: &[u8]) {
    let _ = bluetooth.write_bytes(msg);
    let _ = bluetooth.write_bytes(b"\r\n");
}

fn parse_u32_hex(token: &str) -> Option<u32> {
    let token = token.strip_prefix("0x").unwrap_or(token);
    let token = token.strip_prefix("0X").unwrap_or(token);
    u32::from_str_radix(token, 16).ok()
}

fn decode_hex(input: &[u8], out: &mut [u8]) -> Option<usize> {
    if input.len() % 2 != 0 {
        return None;
    }

    let bytes = input.len() / 2;
    if bytes > out.len() {
        return None;
    }

    for i in 0..bytes {
        let hi = hex_nibble(input[i * 2])?;
        let lo = hex_nibble(input[i * 2 + 1])?;
        out[i] = (hi << 4) | lo;
    }

    Some(bytes)
}

fn hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    }
}

fn append_hex<const N: usize>(dst: &mut String<N>, bytes: &[u8]) {
    for b in bytes {
        let _ = write!(dst, "{:02X}", b);
    }
}

fn decode_control_block(
    flash: &mut crate::FlashDevice,
    start_addr: u32,
    end_addr: u32,
) -> Result<(LogDiveControlDataBlock<MAX_LOG_CONTROL_GASES>, usize), ()> {
    if end_addr <= start_addr || (end_addr - start_addr) < 24 {
        return Err(());
    }

    let mut header = [0u8; 24];
    read_flash_linear(flash, start_addr, &mut header)?;

    let gas_nr = header[23] as usize;
    if gas_nr > MAX_LOG_CONTROL_GASES {
        return Err(());
    }

    let total_len = 24 + gas_nr * 3;
    if end_addr - start_addr < total_len as u32 {
        return Err(());
    }

    let mut bytes = [0u8; MAX_LOG_CONTROL_BYTES];
    bytes[..24].copy_from_slice(&header);
    if gas_nr > 0 {
        read_flash_linear(
            flash,
            start_addr + 24,
            &mut bytes[24..total_len],
        )?;
    }

    let (control, consumed) = LogDiveControlDataBlock::<MAX_LOG_CONTROL_GASES>::from_bytes(&bytes[..total_len])
        .ok_or(())?;

    Ok((control, consumed))
}

fn decode_log_point(
    flash: &mut crate::FlashDevice,
    start_addr: u32,
    end_addr: u32,
) -> Result<(LogPointData, usize), ()> {
    if end_addr <= start_addr || (end_addr - start_addr) < LOG_POINT_BASIC_BYTES as u32 {
        return Err(());
    }

    let remaining = (end_addr - start_addr) as usize;
    let read_len = core::cmp::min(remaining, MAX_LOG_POINT_BYTES);
    let mut bytes = [0u8; MAX_LOG_POINT_BYTES];
    read_flash_linear(flash, start_addr, &mut bytes[..read_len])?;

    LogPointData::from_bytes(&bytes[..read_len]).ok_or(())
}

fn erase_flash_range_4k(
    flash: &mut crate::FlashDevice,
    start_addr: u32,
    end_addr_exclusive: u32,
) -> Result<(), ()> {
    let mut addr = start_addr & !0x0FFF;
    let end = (end_addr_exclusive.saturating_add(0x0FFF)) & !0x0FFF;
    while addr < end {
        flash.erase_4k(addr).map_err(|_| ())?;
        addr = addr.saturating_add(4096);
    }
    Ok(())
}

fn read_flash_linear(
    flash: &mut crate::FlashDevice,
    start_addr: u32,
    out: &mut [u8],
) -> Result<(), ()> {
    for (idx, slot) in out.iter_mut().enumerate() {
        let data = flash.read_bytes::<1>(start_addr + idx as u32).map_err(|_| ())?;
        *slot = data[0];
    }
    Ok(())
}

fn write_flash_linear(
    flash: &mut crate::FlashDevice,
    start_addr: u32,
    bytes: &[u8],
) -> Result<(), ()> {
    if start_addr + bytes.len() as u32 > FW_SLOT_END_EXCLUSIVE {
        return Err(());
    }

    for (idx, b) in bytes.iter().enumerate() {
        flash
            .write_bytes::<1>(start_addr + idx as u32, &[*b])
            .map_err(|_| ())?;
    }
    Ok(())
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
