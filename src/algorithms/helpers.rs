use num::Unsigned;
use stm32l4xx_hal::datetime::{Date, Time};

const fn is_leap_year(year: u32) -> bool {
    (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)
}

const fn days_in_month(year: u32, month: u32) -> u32 {
    match month {
        1 => 31,
        2 => {
            if is_leap_year(year) {
                29
            } else {
                28
            }
        }
        3 => 31,
        4 => 30,
        5 => 31,
        6 => 30,
        7 => 31,
        8 => 31,
        9 => 30,
        10 => 31,
        11 => 30,
        12 => 31,
        _ => 0,
    }
}

const MIN_YEAR: u32 = 1970;
const MAX_YEAR: u32 = 2035;
const SECONDS_IN_DAY: u32 = 86400;

const YEAR_COUNT: usize = (MAX_YEAR - MIN_YEAR + 1) as usize;

const SECONDS_PER_YEAR: [u32; YEAR_COUNT] = {
    let mut arr = [0; YEAR_COUNT];
    let mut i = 0;
    while i < YEAR_COUNT {
        let year = MIN_YEAR + i as u32;
        arr[i] = if is_leap_year(year) {
            366 * SECONDS_IN_DAY
        } else {
            365 * SECONDS_IN_DAY
        };
        i += 1;
    }
    arr
};

pub fn datetime_to_epoch_seconds(date: Date, time: Time) -> u32 {
    if date.year < MIN_YEAR || date.year > MAX_YEAR {
        panic!("Year out of bounds, this should not be reachable")
    }
    let mut days = SECONDS_PER_YEAR[date.year as usize - 1970];
    for m in 1..date.month as u32 {
        days += days_in_month(date.year, m);
    }

    days += (date.day as u32) - 1;
    let seconds = days * 86400 + time.hours * 3600 + time.minutes * 60 + time.seconds;

    seconds
}
