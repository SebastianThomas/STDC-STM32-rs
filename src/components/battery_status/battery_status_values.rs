/** Typically 4.2 or 4.35 */
pub const CHARGE_VOLTAGE: f32 = 4.2;
/** R = 7; H = 30 */
pub const R_SENSE_INTERNAL_MOHM: u16 = 7;
pub const CAP_MAH: u16 = 3000;
/** Typical 100mA, depends on PCB layout and components */
pub const TERM_CURRENT_MA: u16 = 100;
pub const DESIGN_CAP: u16 = (CAP_MAH * R_SENSE_INTERNAL_MOHM) / 5;
pub const ICHG_TERM: u16 =
    ((TERM_CURRENT_MA as u32 * R_SENSE_INTERNAL_MOHM as u32 * 10_000_u32) / 15_625) as u16;
/** [15:7] = Recovery Voltage: 3.88V, [6:0] = Empty: 3.0V */
pub const VEMPTY: u16 = 0x9659;

#[cfg(all(test, not(target_os = "none")))]
mod test {
    use super::*;

    #[test]
    fn test_battery_status_value_design_cap() {
        const DESIGN_CAP_EXPECTED: u16 = 4200;
        assert_eq!(DESIGN_CAP_EXPECTED, DESIGN_CAP);
    }

    #[test]
    fn test_battery_status_value_ichg_term() {
        const ICHG_TERM_EXPECTED: u16 = 448;
        assert_eq!(ICHG_TERM_EXPECTED, ICHG_TERM);
    }

    #[test]
    fn test_battery_status_value_vempty() {
        const VEMPTY_EXPECTED: u16 = 0x9659;
        assert_eq!(VEMPTY_EXPECTED, VEMPTY);
    }
}
