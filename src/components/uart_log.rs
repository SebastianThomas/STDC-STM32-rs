use stm32l4xx_hal::hal::serial::{Read, Write};

pub trait ExternalLogger {
    type TX: Write<u8>;
    type RX: Read<u8>;

    fn log_bytes(&mut self, msg: &[u8]) -> Result<(), <Self::TX as Write<u8>>::Error>;
}

pub struct UartLogger<TX: Write<u8>, RX: Read<u8>> {
    uart_tx: TX,
    _uart_rx: RX,
}

impl<TX: Write<u8>, RX: Read<u8>> UartLogger<TX, RX> {
    pub fn new(uart_tx: TX, uart_rx: RX) -> UartLogger<TX, RX> {
        UartLogger {
            uart_tx,
            _uart_rx: uart_rx,
        }
    }
}

impl<TX: Write<u8>, RX: Read<u8>> ExternalLogger for UartLogger<TX, RX> {
    type TX = TX;
    type RX = RX;

    fn log_bytes(&mut self, msg: &[u8]) -> Result<(), TX::Error> {
        for b in msg {
            let write = nb::block!(self.uart_tx.write(*b));
            if let Err(e) = write {
                return Err(e);
            }
        }
        Ok(())
    }
}
