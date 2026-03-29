use stm32l4xx_hal::{
    hal::{blocking::spi::Write, digital::v2::OutputPin},
    spi::Spi,
};
use thalmann::pressure_unit::{Pressure, msw};

use core::time::Duration;

use crate::components::{
    display::{DisplayState, LedDisplay, spi::SpiDisplay},
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

const STATUS_SCALE: u8 = 2;
const STATUS_Y: u8 = 2;
const STATUS_AREA_HEIGHT: u8 = FONT_HEIGHT * STATUS_SCALE + 4;
const STATUS_PADDING: u8 = 2;
const STATUS_LEFT_X: u8 = 0;
const STATUS_LEFT_W: u8 = SSD1353_WIDTH / 2;
const STATUS_RIGHT_X: u8 = SSD1353_WIDTH / 2;
const STATUS_RIGHT_W: u8 = SSD1353_WIDTH - STATUS_RIGHT_X;

fn append_u32_ascii(mut value: u32, out: &mut [u8], offset: &mut usize) {
    if *offset >= out.len() {
        return;
    }

    if value == 0 {
        out[*offset] = b'0';
        *offset += 1;
        return;
    }

    let mut digits = [0u8; 10];
    let mut pos = 0;
    while value > 0 && pos < digits.len() {
        digits[pos] = (value % 10) as u8;
        value /= 10;
        pos += 1;
    }

    while pos > 0 && *offset < out.len() {
        pos -= 1;
        out[*offset] = b'0' + digits[pos];
        *offset += 1;
    }
}

fn format_depth_text(depth: msw, out: &mut [u8; 16]) -> usize {
    let mut val = depth.to_f32();
    if !val.is_finite() || val < 0.0 {
        val = 0.0;
    }

    let tenths = (val * 10.0 + 0.5) as u32;
    let whole = tenths / 10;
    let frac = tenths % 10;

    let mut len = 0;
    append_u32_ascii(whole, out, &mut len);
    if len < out.len() {
        out[len] = b'.';
        len += 1;
    }
    if len < out.len() {
        out[len] = b'0' + frac as u8;
        len += 1;
    }
    if len < out.len() {
        out[len] = b'm';
        len += 1;
    }

    len
}

fn format_dive_time_text(dive_time: Duration, out: &mut [u8; 16]) -> usize {
    let secs = dive_time.as_secs();
    let mins = (secs / 60) as u32;
    let rem_secs = (secs % 60) as u32;

    let mut len = 0;
    append_u32_ascii(mins, out, &mut len);
    if len < out.len() {
        out[len] = b':';
        len += 1;
    }
    if len + 1 < out.len() {
        out[len] = b'0' + (rem_secs / 10) as u8;
        out[len + 1] = b'0' + (rem_secs % 10) as u8;
        len += 2;
    }

    len
}

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

    fn draw_scaled_text_at(
        &mut self,
        x: u8,
        y: u8,
        text: &[u8],
        scale: u8,
        color: u16,
        max_width: u8,
    ) -> Result<(), SpiError> {
        if scale == 0 {
            return Err(SpiError::new(0, "Text scale must be greater than zero"));
        }
        let char_advance = (FONT_WIDTH + FONT_SPACING) as u16 * scale as u16;
        let max_chars = (max_width as u16 / char_advance) as usize;
        let draw_len = text.len().min(max_chars);

        for (idx, byte) in text.iter().take(draw_len).enumerate() {
            let dx = x as u16 + idx as u16 * char_advance;
            self.draw_scaled_char(dx as u8, y, *byte, scale, color)?;
        }

        Ok(())
    }

    pub fn ssd1353_show_depth_and_dive_time(
        &mut self,
        state: &DisplayState,
    ) -> Result<(), SpiError> {
        let mut depth_label = [0u8; 16];
        let mut time_label = [0u8; 16];
        let depth_len = format_depth_text(state.depth, &mut depth_label);
        let time_len = format_dive_time_text(state.dive_time, &mut time_label);

        let depth_changed = self.update_depth_cache(&depth_label[..depth_len]);
        let time_changed = self.update_time_cache(&time_label[..time_len]);

        if depth_changed {
            self.ssd1353_fill_rect(
                STATUS_LEFT_X,
                0,
                STATUS_LEFT_W,
                STATUS_AREA_HEIGHT,
                COLOR_BLACK,
            )?;
            self.draw_scaled_text_at(
                STATUS_LEFT_X + STATUS_PADDING,
                STATUS_Y,
                &depth_label[..depth_len],
                STATUS_SCALE,
                COLOR_SUBTITLE,
                STATUS_LEFT_W - STATUS_PADDING * 2,
            )?;
        }

        if time_changed {
            self.ssd1353_fill_rect(
                STATUS_RIGHT_X,
                0,
                STATUS_RIGHT_W,
                STATUS_AREA_HEIGHT,
                COLOR_BLACK,
            )?;

            let time_draw_len = time_len.min(
                (STATUS_RIGHT_W as u16 / ((FONT_WIDTH + FONT_SPACING) as u16 * STATUS_SCALE as u16))
                    as usize,
            );
            let time_width = measure_text_width(time_draw_len, STATUS_SCALE);
            let max_x = STATUS_RIGHT_X as u16 + STATUS_RIGHT_W as u16 - STATUS_PADDING as u16;
            let start_x = if time_width + STATUS_PADDING as u16 > STATUS_RIGHT_W as u16 {
                STATUS_RIGHT_X
            } else {
                (max_x - time_width) as u8
            };

            self.draw_scaled_text_at(
                start_x,
                STATUS_Y,
                &time_label[..time_len],
                STATUS_SCALE,
                COLOR_SUBTITLE,
                STATUS_RIGHT_W - STATUS_PADDING * 2,
            )?;
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

    fn refresh_with_state(&mut self, state: &DisplayState) -> Result<(), Self::Error> {
        self.ssd1353_show_depth_and_dive_time(state)
    }
}
