use stm32l4xx_hal::{
    hal::{blocking::spi::Write, digital::v2::OutputPin},
    spi::Spi,
};

use crate::components::{
    display::{LedDisplay, spi::SpiDisplay},
    spi_utils::SpiError,
};

pub const SSD1353_WIDTH: u8 = 160;
pub const SSD1353_HEIGHT: u8 = 128;

const CMD_SET_COLUMN_ADDRESS: u8 = 0x15;
const CMD_SET_ROW_ADDRESS: u8 = 0x75;
const CMD_WRITE_RAM: u8 = 0x5C;
const CMD_SET_REMAP: u8 = 0xA0;
const CMD_SET_DISPLAY_START_LINE: u8 = 0xA1;
const CMD_SET_DISPLAY_OFFSET: u8 = 0xA2;
const CMD_SET_NORMAL_DISPLAY: u8 = 0xA6;
const CMD_FUNCTION_SELECTION: u8 = 0xAB;
const CMD_SLEEP_MODE_OFF: u8 = 0xAF;
const CMD_SLEEP_MODE_ON: u8 = 0xAE;
const CMD_CLOCK_DIVIDER: u8 = 0xB3;
const CMD_PRECHARGE_A: u8 = 0x8A;
const CMD_PRECHARGE_B: u8 = 0x8B;
const CMD_PRECHARGE_C: u8 = 0x8C;
const CMD_PRECHARGE_LEVEL: u8 = 0xBB;
const CMD_VCOMH: u8 = 0xBE;
const CMD_CONTRAST_ABC: u8 = 0xC1;
const CMD_MASTER_CONTRAST: u8 = 0xC7;

const COLOR_BLACK: u16 = 0x0000;
const COLOR_TITLE: u16 = 0x07FF;
const COLOR_SUBTITLE: u16 = 0xFFFF;

const FONT_WIDTH: u8 = 5;
const FONT_HEIGHT: u8 = 7;
const FONT_SPACING: u8 = 1;

const DISPLAY_TITLE: &[u8] = b"STDC";

fn to_upper_ascii(byte: u8) -> u8 {
    if byte.is_ascii_lowercase() {
        byte - 32
    } else {
        byte
    }
}

fn glyph_5x7(byte: u8) -> [u8; FONT_WIDTH as usize] {
    match to_upper_ascii(byte) {
        b' ' => [0x00, 0x00, 0x00, 0x00, 0x00],
        b'-' => [0x08, 0x08, 0x08, 0x08, 0x08],
        b'_' => [0x40, 0x40, 0x40, 0x40, 0x40],
        b'.' => [0x00, 0x60, 0x60, 0x00, 0x00],
        b'0' => [0x3E, 0x51, 0x49, 0x45, 0x3E],
        b'1' => [0x00, 0x42, 0x7F, 0x40, 0x00],
        b'2' => [0x62, 0x51, 0x49, 0x49, 0x46],
        b'3' => [0x22, 0x41, 0x49, 0x49, 0x36],
        b'4' => [0x18, 0x14, 0x12, 0x7F, 0x10],
        b'5' => [0x2F, 0x49, 0x49, 0x49, 0x31],
        b'6' => [0x3E, 0x49, 0x49, 0x49, 0x32],
        b'7' => [0x01, 0x71, 0x09, 0x05, 0x03],
        b'8' => [0x36, 0x49, 0x49, 0x49, 0x36],
        b'9' => [0x26, 0x49, 0x49, 0x49, 0x3E],
        b'A' => [0x7E, 0x11, 0x11, 0x11, 0x7E],
        b'B' => [0x7F, 0x49, 0x49, 0x49, 0x36],
        b'C' => [0x3E, 0x41, 0x41, 0x41, 0x22],
        b'D' => [0x7F, 0x41, 0x41, 0x22, 0x1C],
        b'E' => [0x7F, 0x49, 0x49, 0x49, 0x41],
        b'F' => [0x7F, 0x09, 0x09, 0x09, 0x01],
        b'G' => [0x3E, 0x41, 0x49, 0x49, 0x7A],
        b'H' => [0x7F, 0x08, 0x08, 0x08, 0x7F],
        b'I' => [0x00, 0x41, 0x7F, 0x41, 0x00],
        b'J' => [0x20, 0x40, 0x41, 0x3F, 0x01],
        b'K' => [0x7F, 0x08, 0x14, 0x22, 0x41],
        b'L' => [0x7F, 0x40, 0x40, 0x40, 0x40],
        b'M' => [0x7F, 0x02, 0x0C, 0x02, 0x7F],
        b'N' => [0x7F, 0x04, 0x08, 0x10, 0x7F],
        b'O' => [0x3E, 0x41, 0x41, 0x41, 0x3E],
        b'P' => [0x7F, 0x09, 0x09, 0x09, 0x06],
        b'Q' => [0x3E, 0x41, 0x51, 0x21, 0x5E],
        b'R' => [0x7F, 0x09, 0x19, 0x29, 0x46],
        b'S' => [0x46, 0x49, 0x49, 0x49, 0x31],
        b'T' => [0x01, 0x01, 0x7F, 0x01, 0x01],
        b'U' => [0x3F, 0x40, 0x40, 0x40, 0x3F],
        b'V' => [0x1F, 0x20, 0x40, 0x20, 0x1F],
        b'W' => [0x3F, 0x40, 0x38, 0x40, 0x3F],
        b'X' => [0x63, 0x14, 0x08, 0x14, 0x63],
        b'Y' => [0x03, 0x04, 0x78, 0x04, 0x03],
        b'Z' => [0x61, 0x51, 0x49, 0x45, 0x43],
        _ => [0x7F, 0x41, 0x5D, 0x41, 0x7F],
    }
}

fn measure_text_width(chars: usize, scale: u8) -> u16 {
    if chars == 0 {
        return 0;
    }
    let char_width = (FONT_WIDTH + FONT_SPACING) as u16 * scale as u16;
    char_width * chars as u16 - (FONT_SPACING as u16 * scale as u16)
}

impl<SPI, PINS, EN: OutputPin, RST: OutputPin, NDC: OutputPin> SpiDisplay<SPI, PINS, EN, RST, NDC>
where
    Spi<SPI, PINS>: Write<u8>,
{
    pub fn ssd1353_init(&mut self) -> Result<(), SpiError> {
        self.write_command(CMD_SLEEP_MODE_ON)?;

        self.write_command(CMD_CLOCK_DIVIDER)?;
        self.write_data(&[0xF1])?;

        self.write_command(CMD_SET_REMAP)?;
        self.write_data(&[0x74])?;

        self.write_command(CMD_SET_DISPLAY_START_LINE)?;
        self.write_data(&[0x00])?;

        self.write_command(CMD_SET_DISPLAY_OFFSET)?;
        self.write_data(&[0x00])?;

        self.write_command(CMD_FUNCTION_SELECTION)?;
        self.write_data(&[0x01])?;

        self.write_command(CMD_PRECHARGE_A)?;
        self.write_data(&[0x64])?;
        self.write_command(CMD_PRECHARGE_B)?;
        self.write_data(&[0x78])?;
        self.write_command(CMD_PRECHARGE_C)?;
        self.write_data(&[0x64])?;

        self.write_command(CMD_PRECHARGE_LEVEL)?;
        self.write_data(&[0x3A])?;

        self.write_command(CMD_VCOMH)?;
        self.write_data(&[0x3E])?;

        self.write_command(CMD_CONTRAST_ABC)?;
        self.write_data(&[0x9F, 0x8F, 0x8F])?;

        self.write_command(CMD_MASTER_CONTRAST)?;
        self.write_data(&[0x0F])?;

        self.write_command(CMD_SET_NORMAL_DISPLAY)?;
        self.write_command(CMD_SLEEP_MODE_OFF)
    }

    pub fn ssd1353_set_window(
        &mut self,
        x_start: u8,
        y_start: u8,
        x_end: u8,
        y_end: u8,
    ) -> Result<(), SpiError> {
        if x_start > x_end || y_start > y_end {
            return Err(SpiError::new(0, "Invalid SSD1353 window bounds"));
        }
        if x_end >= SSD1353_WIDTH || y_end >= SSD1353_HEIGHT {
            return Err(SpiError::new(0, "SSD1353 window exceeds display bounds"));
        }

        self.write_command(CMD_SET_COLUMN_ADDRESS)?;
        self.write_data(&[x_start, x_end])?;

        self.write_command(CMD_SET_ROW_ADDRESS)?;
        self.write_data(&[y_start, y_end])
    }

    pub fn ssd1353_write_ram(&mut self, data: &[u8]) -> Result<(), SpiError> {
        self.write_command(CMD_WRITE_RAM)?;
        self.write_data(data)
    }

    pub fn ssd1353_fill(&mut self, color: u16) -> Result<(), SpiError> {
        self.ssd1353_set_window(0, 0, SSD1353_WIDTH - 1, SSD1353_HEIGHT - 1)?;

        let hi = (color >> 8) as u8;
        let lo = (color & 0xFF) as u8;

        self.write_command(CMD_WRITE_RAM)?;
        self.set_data_mode()?;

        let pixel_count = SSD1353_WIDTH as usize * SSD1353_HEIGHT as usize;
        let mut chunk = [0u8; 64];
        for pair in chunk.chunks_exact_mut(2) {
            pair[0] = hi;
            pair[1] = lo;
        }

        let mut remaining = pixel_count;
        while remaining > 0 {
            let pixels_this_round = if remaining > 32 { 32 } else { remaining };
            let bytes = pixels_this_round * 2;
            self.write_data(&chunk[..bytes])?;
            remaining -= pixels_this_round;
        }

        Ok(())
    }

    pub fn ssd1353_clear(&mut self) -> Result<(), SpiError> {
        self.ssd1353_fill(COLOR_BLACK)
    }

    pub fn ssd1353_fill_rect(
        &mut self,
        x: u8,
        y: u8,
        width: u8,
        height: u8,
        color: u16,
    ) -> Result<(), SpiError> {
        if width == 0 || height == 0 {
            return Ok(());
        }
        let x_end = x as u16 + width as u16 - 1;
        let y_end = y as u16 + height as u16 - 1;
        if x_end >= SSD1353_WIDTH as u16 || y_end >= SSD1353_HEIGHT as u16 {
            return Err(SpiError::new(0, "SSD1353 rectangle exceeds display bounds"));
        }

        self.ssd1353_set_window(x, y, x_end as u8, y_end as u8)?;
        self.write_command(CMD_WRITE_RAM)?;
        self.set_data_mode()?;

        let hi = (color >> 8) as u8;
        let lo = (color & 0xFF) as u8;
        let mut row = [0u8; SSD1353_WIDTH as usize * 2];
        let row_bytes = width as usize * 2;
        for pair in row[..row_bytes].chunks_exact_mut(2) {
            pair[0] = hi;
            pair[1] = lo;
        }

        for _ in 0..height {
            self.write_data(&row[..row_bytes])?;
        }

        Ok(())
    }

    fn draw_scaled_char(
        &mut self,
        x: u8,
        y: u8,
        byte: u8,
        scale: u8,
        color: u16,
    ) -> Result<(), SpiError> {
        let glyph = glyph_5x7(byte);

        for (col, col_bits) in glyph.iter().enumerate() {
            for row in 0..FONT_HEIGHT as usize {
                if (col_bits >> row) & 0x01 == 0 {
                    continue;
                }

                let px = x as u16 + (col as u16 * scale as u16);
                let py = y as u16 + (row as u16 * scale as u16);
                self.ssd1353_fill_rect(px as u8, py as u8, scale, scale, color)?;
            }
        }

        Ok(())
    }

    fn draw_scaled_text_centered(
        &mut self,
        y: u8,
        text: &[u8],
        scale: u8,
        color: u16,
    ) -> Result<(), SpiError> {
        if scale == 0 {
            return Err(SpiError::new(0, "Text scale must be greater than zero"));
        }

        let char_advance = (FONT_WIDTH + FONT_SPACING) as u16 * scale as u16;
        let max_chars = SSD1353_WIDTH as usize / char_advance as usize;
        let draw_len = text.len().min(max_chars);
        if draw_len == 0 {
            return Ok(());
        }

        let text_width = measure_text_width(draw_len, scale);
        let start_x = ((SSD1353_WIDTH as u16 - text_width) / 2) as u8;

        for (idx, byte) in text.iter().take(draw_len).enumerate() {
            let x = start_x as u16 + idx as u16 * char_advance;
            self.draw_scaled_char(x as u8, y, *byte, scale, color)?;
        }

        Ok(())
    }
}

impl<SPI, PINS, EN: OutputPin, RST: OutputPin, NDC: OutputPin> LedDisplay
    for SpiDisplay<SPI, PINS, EN, RST, NDC>
where
    Spi<SPI, PINS>: Write<u8>,
{
    type Error = SpiError;

    fn show_splashscreen(&mut self, text: &[u8]) -> Result<(), Self::Error> {
        self.ssd1353_init()?;
        self.ssd1353_clear()?;

        let title_scale = 3;
        let subtitle_scale = 1;
        let title_h = FONT_HEIGHT * title_scale;
        let subtitle_h = FONT_HEIGHT * subtitle_scale;
        let gap = 12;
        let total_h = title_h + gap + subtitle_h;
        let start_y = (SSD1353_HEIGHT - total_h) / 2;

        self.draw_scaled_text_centered(start_y, DISPLAY_TITLE, title_scale, COLOR_TITLE)?;
        self.draw_scaled_text_centered(
            start_y + title_h + gap,
            text,
            subtitle_scale,
            COLOR_SUBTITLE,
        )?;
        Ok(())
    }
}
