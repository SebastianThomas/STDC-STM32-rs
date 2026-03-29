use stm32l4xx_hal::hal::{
    digital::v2::InputPin,
    serial::{Read, Write},
};

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
    ) -> Result<UartBluetoothModule<TX, RX, TXI>, Error<E>> {
        let mut bluetooth = self.enter_command_mode()?;
        bluetooth.set_device_name(bluetooth_name)?;
        bluetooth.start_advertising()?;
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
    /// Requires a guard time (~100 ms silence on UART) before sending "$$$".
    pub fn enter_command_mode(self) -> Result<UartBluetoothCommandMode<TX, RX, TXI>, Error<E>> {
        let mut command_mode = UartBluetoothCommandMode {
            uart_tx: self.uart_tx,
            uart_rx: self.uart_rx,
            tx_ind: self.tx_ind,
        };
        command_mode.write_bytes(b"$$$")?;
        command_mode.wait_for_response(b"CMD>")?;
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
    pub fn exit_command_mode(self) -> Result<UartBluetoothModule<TX, RX, TXI>, Error<E>> {
        let mut normal_mode = UartBluetoothModule {
            uart_tx: self.uart_tx,
            uart_rx: self.uart_rx,
            tx_ind: self.tx_ind,
        };
        normal_mode.write_bytes(b"---")?;
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
        let mut buf = [0u8; 32];
        let n = self.read_line(&mut buf)?;
        if &buf[..n] == b"AOK" {
            Ok(())
        } else {
            Err(Error::InvalidResponse)
        }
    }

    fn wait_for_response(&mut self, expected: &[u8]) -> Result<(), Error<E>> {
        let mut buf = [0u8; 64];
        let n = self.read_line(&mut buf)?;
        if &buf[..n] == expected {
            Ok(())
        } else {
            Err(Error::InvalidResponse)
        }
    }

    fn send_command_with_arg(&mut self, prefix: &[u8], arg: &[u8]) -> Result<(), Error<E>> {
        self.write_bytes(prefix)?;
        self.write_bytes(arg)?;
        self.write_bytes(b"\r")?;
        Ok(())
    }
}
