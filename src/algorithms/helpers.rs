#[cfg(target_os = "none")]
pub use stm32l4xx_hal::datetime::{Date, Time};

#[cfg(not(target_os = "none"))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Date {
    pub day: u32,
    pub date: u32,
    pub month: u32,
    pub year: u32,
}

#[cfg(not(target_os = "none"))]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Time {
    pub hours: u32,
    pub minutes: u32,
    pub seconds: u32,
    pub micros: u32,
    pub daylight_savings: bool,
}

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

const SECONDS_AT_YEAR_START: [u32; YEAR_COUNT] = {
    let mut arr = [0; YEAR_COUNT];
    let mut i = 1;
    while i < YEAR_COUNT {
        arr[i] = arr[i - 1] + SECONDS_PER_YEAR[i - 1];
        i += 1;
    }
    arr
};

pub fn datetime_to_epoch_seconds(date: Date, time: Time) -> u32 {
    if date.year < MIN_YEAR || date.year > MAX_YEAR {
        panic!("Year out of bounds, this should not be reachable")
    }

    let mut seconds = SECONDS_AT_YEAR_START[(date.year - MIN_YEAR) as usize];

    for m in 1..date.month as u32 {
        seconds += days_in_month(date.year, m) * SECONDS_IN_DAY;
    }

    seconds += ((date.date as u32) - 1) * SECONDS_IN_DAY;
    seconds += time.hours * 3600 + time.minutes * 60 + time.seconds;

    seconds
}
