use rtt_target::rprintln;
use stm32l4xx_hal::hal::{
    digital::v2::InputPin,
    serial::{Read, Write},
};

const COMMAND_MODE_RESPONSE_TIMEOUT_POLLS: usize = 250_000;

pub trait BluetoothModule {
    type TX: Write<u8, Error = Self::UartError>;
    type RX: Read<u8, Error = Self::UartError>;
    type TXI: InputPin;
    type UartError;

    fn uart_tx(&mut self) -> &mut Self::TX;
    fn uart_rx(&mut self) -> &mut Self::RX;
    fn tx_ind(&self) -> &Self::TXI;

    fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Error<Self::UartError>> {
        for b in bytes {
            nb::block!(self.uart_tx().write(*b)).map_err(Error::Uart)?;
        }
        Ok(())
    }

    fn read_byte(&mut self) -> Result<u8, Error<Self::UartError>> {
        nb::block!(self.uart_rx().read()).map_err(Error::Uart)
    }

    fn try_read_byte(&mut self) -> Result<Option<u8>, Error<Self::UartError>> {
        match self.uart_rx().read() {
            Ok(b) => Ok(Some(b)),
            Err(nb::Error::WouldBlock) => Ok(None),
            Err(nb::Error::Other(e)) => Err(Error::Uart(e)),
        }
    }

    fn read_line(&mut self, buf: &mut [u8]) -> Result<usize, Error<Self::UartError>> {
        let mut i = 0;
        loop {
            let b = self.read_byte()?;
            if b == b'\r' || i >= buf.len() {
                return Ok(i);
            }
            buf[i] = b;
            i += 1;
        }
    }

    fn is_tx_ready(&self) -> Result<bool, <Self::TXI as InputPin>::Error> {
        self.tx_ind().is_high()
    }
}

#[derive(Debug)]
pub enum Error<E> {
    Uart(E),
    InvalidResponse,
    Timeout,
}

pub struct UartBluetoothModule<TX: Write<u8>, RX: Read<u8>, TXI: InputPin> {
    pub uart_tx: TX,
    pub uart_rx: RX,
    pub tx_ind: TXI,
}

pub struct UartBluetoothCommandMode<TX: Write<u8>, RX: Read<u8>, TXI: InputPin> {
    pub uart_tx: TX,
    pub uart_rx: RX,
    pub tx_ind: TXI,
}

impl<TX, RX, TXI, E> UartBluetoothModule<TX, RX, TXI>
where
    TX: Write<u8, Error = E>,
    RX: Read<u8, Error = E>,
    TXI: InputPin,
{
    pub fn initialize_bluetooth(
        self,
        bluetooth_name: &[u8],
    ) -> Result<UartBluetoothModule<TX, RX, TXI>, (Error<E>, UartBluetoothModule<TX, RX, TXI>)>
    {
        rprintln!("Entering CMD Mode");
        let mut bluetooth = match self.enter_command_mode() {
            Ok(bluetooth) => bluetooth,
            Err((error, bluetooth)) => return Err((error, bluetooth)),
        };
        rprintln!("Setting Device Name");
        if let Err(error) = bluetooth.set_device_name(bluetooth_name) {
            return Err((error, bluetooth.into_module()));
        }
        rprintln!("Starting Advertising");
        if let Err(error) = bluetooth.start_advertising() {
            return Err((error, bluetooth.into_module()));
        }
        rprintln!("Exit CMD Mode");
        bluetooth.exit_command_mode()
    }
}

impl<TX, RX, TXI, E> BluetoothModule for UartBluetoothModule<TX, RX, TXI>
where
    TX: Write<u8, Error = E>,
    RX: Read<u8, Error = E>,
    TXI: InputPin,
{
    type TX = TX;
    type RX = RX;
    type TXI = TXI;
    type UartError = E;

    fn uart_tx(&mut self) -> &mut Self::TX {
        &mut self.uart_tx
    }

    fn uart_rx(&mut self) -> &mut Self::RX {
        &mut self.uart_rx
    }

    fn tx_ind(&self) -> &Self::TXI {
        &self.tx_ind
    }
}

impl<TX, RX, TXI, E> BluetoothModule for UartBluetoothCommandMode<TX, RX, TXI>
where
    TX: Write<u8, Error = E>,
    RX: Read<u8, Error = E>,
    TXI: InputPin,
{
    type TX = TX;
    type RX = RX;
    type TXI = TXI;
    type UartError = E;

    fn uart_tx(&mut self) -> &mut Self::TX {
        &mut self.uart_tx
    }

    fn uart_rx(&mut self) -> &mut Self::RX {
        &mut self.uart_rx
    }

    fn tx_ind(&self) -> &Self::TXI {
        &self.tx_ind
    }
}

impl<TX, RX, TXI, E> UartBluetoothModule<TX, RX, TXI>
where
    TX: Write<u8, Error = E>,
    RX: Read<u8, Error = E>,
    TXI: InputPin,
{
    /// Requires a guard time (~86 ms silence on UART) before sending "$$$".
    pub fn enter_command_mode(
        self,
    ) -> Result<UartBluetoothCommandMode<TX, RX, TXI>, (Error<E>, UartBluetoothModule<TX, RX, TXI>)>
    {
        let mut command_mode = UartBluetoothCommandMode {
            uart_tx: self.uart_tx,
            uart_rx: self.uart_rx,
            tx_ind: self.tx_ind,
        };
        rprintln!("Writing Bytes");
        if let Err(error) = command_mode.write_bytes(b"$$$") {
            return Err((error, command_mode.into_module()));
        }
        rprintln!("Wrote Start Bytes");
        if let Err(error) = command_mode.wait_for_response(b"CMD>") {
            return Err((error, command_mode.into_module()));
        }
        rprintln!("Found CMD Response");
        Ok(command_mode)
    }

    pub fn write_data(&mut self, data: &[u8]) -> Result<(), Error<E>> {
        self.write_bytes(data)
    }

    pub fn read_data(&mut self) -> Result<u8, Error<E>> {
        self.read_byte()
    }
}

impl<TX, RX, TXI, E> UartBluetoothCommandMode<TX, RX, TXI>
where
    TX: Write<u8, Error = E>,
    RX: Read<u8, Error = E>,
    TXI: InputPin,
{
    fn into_module(self) -> UartBluetoothModule<TX, RX, TXI> {
        UartBluetoothModule {
            uart_tx: self.uart_tx,
            uart_rx: self.uart_rx,
            tx_ind: self.tx_ind,
        }
    }

    pub fn exit_command_mode(
        self,
    ) -> Result<UartBluetoothModule<TX, RX, TXI>, (Error<E>, UartBluetoothModule<TX, RX, TXI>)>
    {
        let mut normal_mode = UartBluetoothModule {
            uart_tx: self.uart_tx,
            uart_rx: self.uart_rx,
            tx_ind: self.tx_ind,
        };
        if let Err(error) = normal_mode.write_bytes(b"---") {
            return Err((error, normal_mode));
        }
        Ok(normal_mode)
    }

    pub fn reboot(&mut self) -> Result<(), Error<E>> {
        self.send_command("R,1")?;
        Ok(())
    }

    pub fn set_device_name(&mut self, name: &[u8]) -> Result<(), Error<E>> {
        self.send_command_with_arg(b"SN,", name)?;
        self.wait_for_ok()
    }

    pub fn start_advertising(&mut self) -> Result<(), Error<E>> {
        self.send_command("A")?;
        self.wait_for_ok()
    }

    pub fn disconnect(&mut self) -> Result<(), Error<E>> {
        self.send_command("K,1")?;
        self.wait_for_ok()
    }

    fn send_command(&mut self, cmd: &str) -> Result<(), Error<E>> {
        self.write_bytes(cmd.as_bytes())?;
        self.write_bytes(b"\r")?;
        Ok(())
    }

    fn wait_for_ok(&mut self) -> Result<(), Error<E>> {
        self.wait_for_response(b"AOK")
    }

    fn wait_for_response(&mut self, expected: &[u8]) -> Result<(), Error<E>> {
        let mut buf = [0u8; 64];
        let mut len = 0usize;
        let mut idle_polls = 0usize;

        loop {
            match self.try_read_byte()? {
                Some(b) => {
                    idle_polls = 0;

                    if b == b'\r' {
                        return if &buf[..len] == expected {
                            Ok(())
                        } else {
                            Err(Error::InvalidResponse)
                        };
                    }

                    if len >= buf.len() {
                        return Err(Error::InvalidResponse);
                    }

                    buf[len] = b;
                    len += 1;
                }
                None => {
                    idle_polls += 1;
                    if idle_polls >= COMMAND_MODE_RESPONSE_TIMEOUT_POLLS {
                        return Err(Error::Timeout);
                    }
                }
            }
        }
    }

    fn send_command_with_arg(&mut self, prefix: &[u8], arg: &[u8]) -> Result<(), Error<E>> {
        self.write_bytes(prefix)?;
        self.write_bytes(arg)?;
        self.write_bytes(b"\r")?;
        Ok(())
    }
}
