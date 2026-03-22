use stm32l4xx_hal::hal::{
    digital::v2::InputPin,
    serial::{Read, Write},
};

pub trait BluetoothModule {
    type TX: Write<u8>;
    type RX: Read<u8>;
}

pub struct UartBluetoothModule<TX: Write<u8>, RX: Read<u8>, TXI: InputPin> {
    uart_tx: TX,
    uart_rx: RX,
    tx_ind: TXI,
}

impl<TX: Write<u8>, RX: Read<u8>, TXI: InputPin> UartBluetoothModule<TX, RX, TXI> {
    pub fn new(uart_tx: TX, uart_rx: RX, tx_ind: TXI) -> UartBluetoothModule<TX, RX, TXI> {
        UartBluetoothModule {
            uart_tx,
            uart_rx,
            tx_ind,
        }
    }
}
