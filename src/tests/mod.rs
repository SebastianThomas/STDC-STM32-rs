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

#[cfg(all(test, not(target_os = "none")))]
mod bluetooth_serialization_tests {
    use crate::dive_log_host::{
        LevelState, LogDiveControlDataBlock, LogPointData, LogPointMetadata,
    };
    use thalmann::{
        dive::DiveMeasurement,
        gas::GasMix,
        pressure_unit::{Pa, Pressure},
    };

    #[test]
    fn control_block_roundtrip_from_new_and_from_bytes() {
        let gases = [
            GasMix::new(0.21, 0.0).expect("valid gas"),
            GasMix::new(0.32, 0.0).expect("valid gas"),
        ];

        let block = LogDiveControlDataBlock::<2>::new(1_700_000_000, 1234, 42, 1013, 23, 4, &gases);
        let raw = block.raw_bytes();
        let (decoded, consumed) =
            LogDiveControlDataBlock::<2>::from_bytes(&raw).expect("decode control block");

        assert_eq!(consumed, 24 + 2 * 3);
        assert_eq!(decoded.raw_bytes(), raw);
        assert_eq!(decoded.gas_content_bytes(), block.gas_content_bytes());
    }

    #[test]
    fn control_block_decode_rejects_short_payload() {
        let too_short = [0u8; 23];
        assert!(LogDiveControlDataBlock::<1>::from_bytes(&too_short).is_none());
    }

    #[test]
    fn log_point_roundtrip_basic_and_depth_quantization() {
        let metadata = LogPointMetadata::new(true, false, LevelState::Descending, [false; 4]);
        let measurement = DiveMeasurement {
            time_ms: 12_345,
            depth: Pa::new(114_321.0),
            gas: 0,
        };

        let point = LogPointData::new(metadata, &measurement, 7, 19, 88);
        let basic_bytes = point.basic_bytes();
        let (decoded, consumed) = LogPointData::from_bytes(&basic_bytes).expect("decode basic");

        assert_eq!(consumed, 8);
        assert!(decoded.deco_data().is_none());
        assert_eq!(decoded.basic_data().metadata().byte(), metadata.byte());

        let original_depth_m = measurement.depth.to_msw().to_f32();
        let decoded_depth_m = decoded.basic_data().depth().to_num::<f32>();
        let quantization_step = 1.0 / 64.0;
        assert!((decoded_depth_m - original_depth_m).abs() <= quantization_step);
    }

    #[test]
    fn log_point_decode_with_deco_payload_is_lossless() {
        let metadata = LogPointMetadata::new(
            true,
            true,
            LevelState::Ascending,
            [false, false, false, true],
        );
        let mut bytes = [0u8; 16];
        bytes[0] = 0x00;
        bytes[1] = 0x3C;
        bytes[2] = 0x01;
        bytes[3] = 0x80;
        bytes[4] = metadata.byte();
        bytes[5] = 77;
        bytes[6] = 0xF2;
        bytes[7] = 5;
        bytes[8..16].copy_from_slice(&[10, 11, 12, 13, 14, 15, 16, 17]);

        let (decoded, consumed) = LogPointData::from_bytes(&bytes).expect("decode deco");

        assert_eq!(consumed, 16);
        assert!(decoded.deco_data().is_some());
        assert_eq!(decoded.basic_bytes(), bytes[..8]);
        assert_eq!(decoded.deco_bytes().expect("deco bytes"), bytes[8..16]);
    }

    #[test]
    fn log_point_decode_rejects_truncated_deco_payload() {
        let metadata =
            LogPointMetadata::new(false, false, LevelState::Level, [false, false, false, true]);
        let mut bytes = [0u8; 12];
        bytes[4] = metadata.byte();
        assert!(LogPointData::from_bytes(&bytes).is_none());
    }
}
