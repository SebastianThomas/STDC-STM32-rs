#[cfg(all(test, not(target_os = "none")))]
mod datetime_helpers_tests {
    use std::time::{Duration, UNIX_EPOCH};

    use crate::algorithms::helpers::{Date, Time, datetime_to_epoch_seconds};

    fn assert_epoch(date: Date, time: Time, expected_epoch: u32) {
        let got = datetime_to_epoch_seconds(date, time);
        assert_eq!(got, expected_epoch);

        let got_system_time = UNIX_EPOCH + Duration::from_secs(got as u64);
        let expected_system_time = UNIX_EPOCH + Duration::from_secs(expected_epoch as u64);
        assert_eq!(got_system_time, expected_system_time);
    }

    #[test]
    fn unix_epoch_start() {
        assert_epoch(
            Date {
                day: 4,
                date: 1,
                month: 1,
                year: 1970,
            },
            Time {
                hours: 0,
                minutes: 0,
                seconds: 0,
                micros: 0,
                daylight_savings: false,
            },
            0,
        );
    }

    #[test]
    fn leap_day_2000_midnight() {
        // 2000-02-29T00:00:00Z
        assert_epoch(
            Date {
                day: 2,
                date: 29,
                month: 2,
                year: 2000,
            },
            Time {
                hours: 0,
                minutes: 0,
                seconds: 0,
                micros: 0,
                daylight_savings: false,
            },
            951782400,
        );
    }

    #[test]
    fn current_project_boot_datetime() {
        // 2025-11-10T00:00:00Z
        assert_epoch(
            Date {
                day: 1,
                date: 10,
                month: 11,
                year: 2025,
            },
            Time {
                hours: 0,
                minutes: 0,
                seconds: 0,
                micros: 0,
                daylight_savings: false,
            },
            1762732800,
        );
    }

    #[test]
    fn arbitrary_datetime_with_time_of_day() {
        // 2024-03-01T12:34:56Z
        assert_epoch(
            Date {
                day: 5,
                date: 1,
                month: 3,
                year: 2024,
            },
            Time {
                hours: 12,
                minutes: 34,
                seconds: 56,
                micros: 0,
                daylight_savings: false,
            },
            1709296496,
        );
    }
}
