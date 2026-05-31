use core::fmt::Debug;

#[allow(unused_imports)]
use stdc_diving_algorithms::pressure_unit::{Pa, Pressure, msw};

const DIVE_DECO_UPDATE_MIN_INTERVAL_MILLIS: u32 = 5_000;
#[cfg_attr(feature = "fixed_rate_algorithms", allow(dead_code))]
const DIVE_DECO_UPDATE_MAX_INTERVAL_MILLIS: u32 = 30_000;
#[cfg_attr(feature = "fixed_rate_algorithms", allow(dead_code))]
const DIVE_DECO_SPEED_REFERENCE_M_PER_S: f32 = 0.60;
#[cfg_attr(feature = "fixed_rate_algorithms", allow(dead_code))]
const DIVE_DECO_DEPTH_REFERENCE_M: f32 = 40.0;
const DIVE_O2TOX_UPDATE_MIN_INTERVAL_MILLIS: u32 = 10_000;
#[cfg_attr(feature = "fixed_rate_algorithms", allow(dead_code))]
const DIVE_O2TOX_UPDATE_MAX_INTERVAL_MILLIS: u32 = 120_000;
#[cfg_attr(feature = "fixed_rate_algorithms", allow(dead_code))]
const DIVE_O2TOX_DEPTH_REFERENCE_M: f32 = 60.0;

pub trait RateAlgorithm<P, I, Output> {
    type Error: Debug;
    fn next_iter(&mut self, prev: P, instance: I) -> Result<Output, Self::Error>;
}

#[derive(Clone, Debug)]
pub struct DiffAimdRateAlgorithm<const WINDOW: usize> {
    aimd: AimdRateAlgorithm,
    window_values: [f32; WINDOW],
    window_intervals: [u32; WINDOW],
    current_window_size: usize,
    next_window_insert_index: usize,
    change_rate_reference_per_s: f32,
    change_detection_threshold_ratio: f32,
    min_value: u32,
    max_value: u32,
}

impl<const WINDOW: usize> DiffAimdRateAlgorithm<WINDOW> {
    pub fn new(
        min_value: u32,
        max_value: u32,
        change_rate_reference_per_s: f32,
        aimd_initial: u32,
        aimd_max: u32,
        aimd_ai_millis: u32,
    ) -> Self {
        Self {
            aimd: AimdRateAlgorithm::new::<3, 4>(aimd_initial, aimd_max, aimd_ai_millis),
            window_values: [0.0; WINDOW],
            window_intervals: [0; WINDOW],
            current_window_size: 0,
            next_window_insert_index: 0,
            change_rate_reference_per_s,
            change_detection_threshold_ratio: 0.15,
            min_value,
            max_value,
        }
    }

    fn next_interval_with_forced_change_signal(
        &mut self,
        prev: (f32, u32),
        instance: f32,
        force_change_signal: bool,
    ) -> Result<u32, ()> {
        let (previous_value, elapsed_millis) = prev;

        self.window_values[self.next_window_insert_index] = previous_value;
        self.window_intervals[self.next_window_insert_index] = elapsed_millis;
        self.next_window_insert_index = (self.next_window_insert_index + 1) % WINDOW;
        if self.current_window_size < WINDOW {
            self.current_window_size += 1;
        }

        let oldest_index = if self.current_window_size < WINDOW {
            0
        } else {
            self.next_window_insert_index
        };
        let mut total_elapsed_millis = 0u32;
        for i in 0..self.current_window_size {
            total_elapsed_millis = total_elapsed_millis
                .saturating_add(self.window_intervals[(oldest_index + i) % WINDOW]);
        }

        let elapsed_seconds = (total_elapsed_millis.max(1) as f32) / 1000.0;
        let oldest = self.window_values[oldest_index];
        let diff_per_s = (instance - oldest).abs() / elapsed_seconds;
        let speed_factor = (diff_per_s / self.change_rate_reference_per_s).clamp(0.0, 1.0);
        let change_by_diff = speed_factor > self.change_detection_threshold_ratio;

        let change = change_by_diff || force_change_signal;

        let interval = self.aimd.next_iter(change).unwrap_or(self.max_value);
        if interval < self.min_value {
            return Ok(self.min_value);
        }
        if interval > self.max_value {
            return Ok(self.max_value);
        }
        Ok(interval)
    }
}

impl<const WINDOW: usize> RateAlgorithm<(f32, u32), f32, u32> for DiffAimdRateAlgorithm<WINDOW> {
    type Error = ();

    fn next_iter(&mut self, prev: (f32, u32), instance: f32) -> Result<u32, Self::Error> {
        self.next_interval_with_forced_change_signal(prev, instance, false)
    }
}

#[derive(Clone, Debug)]
pub struct DynamicDiffAimdRateAlgorithm<const WINDOW: usize> {
    diff_aimd: DiffAimdRateAlgorithm<WINDOW>,
    absolute_reference: f32,
    absolute_trigger_ratio: f32,
}

impl<const WINDOW: usize> DynamicDiffAimdRateAlgorithm<WINDOW> {
    pub fn new(
        min_value: u32,
        max_value: u32,
        change_rate_reference_per_s: f32,
        absolute_reference: f32,
        aimd_initial: u32,
        aimd_max: u32,
        aimd_ai_millis: u32,
    ) -> Self {
        Self {
            diff_aimd: DiffAimdRateAlgorithm::new(
                min_value,
                max_value,
                change_rate_reference_per_s,
                aimd_initial,
                aimd_max,
                aimd_ai_millis,
            ),
            absolute_reference,
            absolute_trigger_ratio: 0.8,
        }
    }
}

impl<const WINDOW: usize> RateAlgorithm<(f32, u32), f32, u32>
    for DynamicDiffAimdRateAlgorithm<WINDOW>
{
    type Error = ();

    fn next_iter(&mut self, prev: (f32, u32), instance: f32) -> Result<u32, Self::Error> {
        let absolute_factor = (instance.abs() / self.absolute_reference).clamp(0.0, 1.0);
        let change_by_absolute = absolute_factor > self.absolute_trigger_ratio;
        self.diff_aimd
            .next_interval_with_forced_change_signal(prev, instance, change_by_absolute)
    }
}

#[cfg(not(feature = "fixed_rate_algorithms"))]
#[derive(Clone, Debug)]
pub struct DecoUpdateRateAlgorithm(DynamicDiffAimdRateAlgorithm<5>);

#[cfg(feature = "fixed_rate_algorithms")]
#[derive(Clone, Debug)]
pub struct DecoUpdateRateAlgorithm(FixedRateAlgorithm);

#[cfg(not(feature = "fixed_rate_algorithms"))]
impl Default for DecoUpdateRateAlgorithm {
    fn default() -> Self {
        DecoUpdateRateAlgorithm(DynamicDiffAimdRateAlgorithm::new(
            DIVE_DECO_UPDATE_MIN_INTERVAL_MILLIS,
            DIVE_DECO_UPDATE_MAX_INTERVAL_MILLIS,
            DIVE_DECO_SPEED_REFERENCE_M_PER_S,
            DIVE_DECO_DEPTH_REFERENCE_M,
            DIVE_DECO_UPDATE_MAX_INTERVAL_MILLIS,
            DIVE_DECO_UPDATE_MAX_INTERVAL_MILLIS,
            1000,
        ))
    }
}

#[cfg(feature = "fixed_rate_algorithms")]
impl Default for DecoUpdateRateAlgorithm {
    fn default() -> Self {
        DecoUpdateRateAlgorithm(FixedRateAlgorithm::new(DIVE_DECO_UPDATE_MIN_INTERVAL_MILLIS))
    }
}

#[cfg(not(feature = "fixed_rate_algorithms"))]
impl RateAlgorithm<(msw, u32), msw, u32> for DecoUpdateRateAlgorithm {
    type Error = ();

    fn next_iter(&mut self, prev: (msw, u32), instance: msw) -> Result<u32, Self::Error> {
        self.0
            .next_iter((prev.0.to_f32(), prev.1), instance.to_f32())
    }
}

#[cfg(feature = "fixed_rate_algorithms")]
impl RateAlgorithm<(msw, u32), msw, u32> for DecoUpdateRateAlgorithm {
    type Error = ();

    fn next_iter(&mut self, prev: (msw, u32), instance: msw) -> Result<u32, Self::Error> {
        self.0.next_iter(prev, instance)
    }
}

#[cfg(not(feature = "fixed_rate_algorithms"))]
#[derive(Clone, Debug)]
pub struct O2ToxUpdateRateAlgorithm(DynamicDiffAimdRateAlgorithm<5>);

#[cfg(feature = "fixed_rate_algorithms")]
#[derive(Clone, Debug)]
pub struct O2ToxUpdateRateAlgorithm(FixedRateAlgorithm);

#[cfg(not(feature = "fixed_rate_algorithms"))]
impl Default for O2ToxUpdateRateAlgorithm {
    fn default() -> Self {
        O2ToxUpdateRateAlgorithm(DynamicDiffAimdRateAlgorithm::new(
            DIVE_O2TOX_UPDATE_MIN_INTERVAL_MILLIS,
            DIVE_O2TOX_UPDATE_MAX_INTERVAL_MILLIS,
            DIVE_DECO_SPEED_REFERENCE_M_PER_S, // reuse speed reference
            DIVE_O2TOX_DEPTH_REFERENCE_M,
            DIVE_O2TOX_UPDATE_MAX_INTERVAL_MILLIS,
            DIVE_O2TOX_UPDATE_MAX_INTERVAL_MILLIS,
            2000,
        ))
    }
}

#[cfg(feature = "fixed_rate_algorithms")]
impl Default for O2ToxUpdateRateAlgorithm {
    fn default() -> Self {
        O2ToxUpdateRateAlgorithm(FixedRateAlgorithm::new(DIVE_O2TOX_UPDATE_MIN_INTERVAL_MILLIS))
    }
}

#[cfg(not(feature = "fixed_rate_algorithms"))]
impl RateAlgorithm<msw, msw, u32> for O2ToxUpdateRateAlgorithm {
    type Error = ();

    fn next_iter(&mut self, prev: msw, instance: msw) -> Result<u32, Self::Error> {
        self.0
            .next_iter((prev.to_f32(), 1000u32), instance.to_f32())
    }
}

#[cfg(not(feature = "fixed_rate_algorithms"))]
impl RateAlgorithm<(msw, u32), msw, u32> for O2ToxUpdateRateAlgorithm {
    type Error = ();

    fn next_iter(&mut self, prev: (msw, u32), instance: msw) -> Result<u32, Self::Error> {
        self.0
            .next_iter((prev.0.to_f32(), prev.1), instance.to_f32())
    }
}

#[cfg(feature = "fixed_rate_algorithms")]
impl RateAlgorithm<msw, msw, u32> for O2ToxUpdateRateAlgorithm {
    type Error = ();

    fn next_iter(&mut self, prev: msw, instance: msw) -> Result<u32, Self::Error> {
        self.0.next_iter(prev, instance)
    }
}

#[cfg(feature = "fixed_rate_algorithms")]
impl RateAlgorithm<(msw, u32), msw, u32> for O2ToxUpdateRateAlgorithm {
    type Error = ();

    fn next_iter(&mut self, prev: (msw, u32), instance: msw) -> Result<u32, Self::Error> {
        self.0.next_iter(prev, instance)
    }
}

#[cfg(not(feature = "fixed_rate_algorithms"))]
pub struct FlashLogRateAlgorithm(DiffAimdRateAlgorithm<5>);

#[cfg(feature = "fixed_rate_algorithms")]
#[derive(Clone, Debug)]
pub struct FlashLogRateAlgorithm(FixedRateAlgorithm);

#[cfg(not(feature = "fixed_rate_algorithms"))]
impl Default for FlashLogRateAlgorithm {
    fn default() -> Self {
        Self(DiffAimdRateAlgorithm::new(
            5_000,
            20_000,
            DIVE_DECO_SPEED_REFERENCE_M_PER_S,
            20_000,
            20_000,
            5_000,
        ))
    }
}

#[cfg(feature = "fixed_rate_algorithms")]
impl Default for FlashLogRateAlgorithm {
    fn default() -> Self {
        Self(FixedRateAlgorithm::new(5_000))
    }
}

#[cfg(not(feature = "fixed_rate_algorithms"))]
impl RateAlgorithm<(Pa, u32), Pa, u32> for FlashLogRateAlgorithm {
    type Error = ();
    fn next_iter(&mut self, prev: (Pa, u32), instance: Pa) -> Result<u32, Self::Error> {
        self.0.next_iter(
            (prev.0.to_msw().to_f32(), prev.1),
            instance.to_msw().to_f32(),
        )
    }
}

#[cfg(feature = "fixed_rate_algorithms")]
impl RateAlgorithm<(Pa, u32), Pa, u32> for FlashLogRateAlgorithm {
    type Error = ();

    fn next_iter(&mut self, prev: (Pa, u32), instance: Pa) -> Result<u32, Self::Error> {
        self.0.next_iter(prev, instance)
    }
}

#[derive(Clone, Debug)]
pub struct FixedRateAlgorithm {
    millis: u32,
}

impl FixedRateAlgorithm {
    pub fn new(millis: u32) -> FixedRateAlgorithm {
        FixedRateAlgorithm { millis }
    }
}

impl<P, I> RateAlgorithm<P, I, u32> for FixedRateAlgorithm {
    type Error = ();
    fn next_iter(&mut self, _prev: P, _instance: I) -> Result<u32, Self::Error> {
        Ok(self.millis)
    }
}

#[derive(Clone, Debug)]
struct AimdRateAlgorithm {
    current: u32,
    max: u32,
    ai_millis: u32,
    md_num: u64,
    md_den: u64,
}

impl AimdRateAlgorithm {
    pub fn new<const MD_NUM: u64, const MD_DEN: u64>(
        initial: u32,
        max: u32,
        ai_millis: u32,
    ) -> AimdRateAlgorithm {
        assert!(MD_NUM > 0);
        assert!(MD_DEN > 0);
        assert!(MD_NUM < MD_DEN);
        AimdRateAlgorithm {
            current: if initial <= max { initial } else { max },
            max,
            ai_millis,
            md_num: MD_NUM,
            md_den: MD_DEN,
        }
    }

    fn next_iter(&mut self, change: bool) -> Result<u32, ()> {
        if change {
            self.current = ((self.current as u64 * self.md_num as u64) / self.md_den as u64) as u32
        } else {
            self.current = self.current.saturating_add(self.ai_millis)
        }
        if self.current > self.max {
            self.current = self.max;
        }
        Ok(self.current)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    fn collect_pressure_rates(
        initial_pressure_pa: f32,
        sample_pressures_pa: &[f32],
        elapsed_millis: u32,
    ) -> Vec<u32> {
        let mut algorithm = FlashLogRateAlgorithm::default();
        let mut previous_pressure = Pa::new(initial_pressure_pa);

        sample_pressures_pa
            .iter()
            .map(|pressure_pa| {
                let current_pressure = Pa::new(*pressure_pa);
                let rate_millis = algorithm
                    .next_iter((previous_pressure, elapsed_millis), current_pressure)
                    .unwrap();
                previous_pressure = current_pressure;
                rate_millis
            })
            .collect()
    }

    #[test]
    fn flash_log_rate_reacts_to_larger_pressure_changes() {
        let rates = collect_pressure_rates(
            100_000.0,
            &[
                100_006.0, 100_013.0, 100_021.0, 100_030.0, 100_040.0, 100_051.0, 100_063.0,
                100_076.0, 100_090.0, 100_105.0,
            ],
            1_000,
        );

        println!("flash log rates (5-sample window, 10 samples): {:?}", rates);
        assert_eq!(rates.len(), 10);
        assert!(rates.iter().all(|rate| *rate >= 5_000 && *rate <= 20_000));
    }

    #[test]
    fn deco_rate_prefers_depth_once_the_profile_gets_deeper() {
        let mut algorithm = DecoUpdateRateAlgorithm::default();
        let sample_depths = [23.5, 23.6, 23.7, 23.8, 23.9, 24.0, 24.1, 24.2, 24.3, 24.4];
        let mut previous_depth = msw::new(23.4);
        let rates: Vec<u32> = sample_depths
            .iter()
            .map(|depth_m| {
                let current_depth = msw::new(*depth_m);
                let rate_millis = algorithm
                    .next_iter((previous_depth, 10_000), current_depth)
                    .unwrap();
                previous_depth = current_depth;
                rate_millis
            })
            .collect();

        println!("deco rates (5-sample window, 10 samples): {:?}", rates);
        assert_eq!(rates.len(), 10);
        assert!(
            rates
                .iter()
                .all(|rate| *rate >= DIVE_DECO_UPDATE_MIN_INTERVAL_MILLIS)
        );
        assert!(
            rates
                .iter()
                .all(|rate| *rate <= DIVE_DECO_UPDATE_MAX_INTERVAL_MILLIS)
        );
    }

    #[test]
    fn o2_rate_prefers_depth_once_the_profile_gets_deeper() {
        let mut algorithm = O2ToxUpdateRateAlgorithm::default();
        let sample_depths = [35.5, 35.6, 35.7, 35.8, 35.9, 36.0, 36.1, 36.2, 36.3, 36.4];
        let mut previous_depth = msw::new(35.4);
        let rates: Vec<u32> = sample_depths
            .iter()
            .map(|depth_m| {
                let current_depth = msw::new(*depth_m);
                let rate_millis = algorithm
                    .next_iter((previous_depth, 10_000), current_depth)
                    .unwrap();
                previous_depth = current_depth;
                rate_millis
            })
            .collect();

        println!("o2 rates (5-sample window, 10 samples): {:?}", rates);
        assert_eq!(rates.len(), 10);
        assert!(
            rates
                .iter()
                .all(|rate| *rate >= DIVE_O2TOX_UPDATE_MIN_INTERVAL_MILLIS)
        );
        assert!(
            rates
                .iter()
                .all(|rate| *rate <= DIVE_O2TOX_UPDATE_MAX_INTERVAL_MILLIS)
        );
    }

    #[test]
    fn flash_log_contrast_fast_vs_slow_change_rate() {
        let slow_samples = [
            100_001.0, 101_202.0, 100_403.0, 100_604.0, 100_805.0, 101_006.0, 102_207.0, 103_408.0,
            104_609.0, 105_010.0,
        ];
        let fast_samples = [
            100_500.0, 101_000.0, 101_500.0, 102_000.0, 102_500.0, 103_000.0, 103_500.0, 104_000.0,
            104_500.0, 105_000.0,
        ];
        let slow_rates = collect_pressure_rates(100_000.0, &slow_samples, 2_000);
        let fast_rates = collect_pressure_rates(100_000.0, &fast_samples, 500);

        println!("flash slow rates: {:?}", slow_rates);
        println!("flash fast rates: {:?}", fast_rates);

        let slow_avg = slow_rates.iter().sum::<u32>() / slow_rates.len() as u32;
        let fast_avg = fast_rates.iter().sum::<u32>() / fast_rates.len() as u32;
        assert!(2 * fast_avg < slow_avg);
    }

    #[test]
    fn flash_log_ignores_absolute_value_offset() {
        let shallow_rates = collect_pressure_rates(
            100_000.0,
            &[
                100_006.0, 100_013.0, 100_021.0, 100_030.0, 100_040.0, 100_051.0, 100_063.0,
                100_076.0, 100_090.0, 100_105.0,
            ],
            1_000,
        );

        let offset_rates = collect_pressure_rates(
            120_000.0,
            &[
                120_006.0, 120_013.0, 120_021.0, 120_030.0, 120_040.0, 120_051.0, 120_063.0,
                120_076.0, 120_090.0, 120_105.0,
            ],
            1_000,
        );

        println!("flash base rates: {:?}", shallow_rates);
        println!("flash offset rates: {:?}", offset_rates);
        assert_eq!(shallow_rates, offset_rates);
    }

    #[test]
    fn deco_contrast_shallow_vs_deep_profile() {
        let mut shallow = DecoUpdateRateAlgorithm::default();
        let mut deep = DecoUpdateRateAlgorithm::default();

        let shallow_depths = [
            5.001, 5.002, 5.003, 5.004, 5.005, 5.006, 5.007, 5.008, 5.009, 5.010,
        ];
        let deep_depths = [45.5, 46.0, 46.5, 47.0, 47.5, 48.0, 48.5, 49.0, 49.5, 50.0];

        let mut prev_shallow = msw::new(5.0);
        let mut prev_deep = msw::new(45.0);
        let shallow_rates: Vec<u32> = shallow_depths
            .iter()
            .map(|d| {
                let cur = msw::new(*d);
                let r = shallow.next_iter((prev_shallow, 1_000), cur).unwrap();
                prev_shallow = cur;
                r
            })
            .collect();
        let deep_rates: Vec<u32> = deep_depths
            .iter()
            .map(|d| {
                let cur = msw::new(*d);
                let r = deep.next_iter((prev_deep, 1_000), cur).unwrap();
                prev_deep = cur;
                r
            })
            .collect();

        println!("deco shallow rates: {:?}", shallow_rates);
        println!("deco deep rates: {:?}", deep_rates);
        let shallow_avg = shallow_rates.iter().sum::<u32>() / shallow_rates.len() as u32;
        let deep_avg = deep_rates.iter().sum::<u32>() / deep_rates.len() as u32;
        assert!(deep_avg < shallow_avg);
    }

    #[test]
    fn o2_contrast_shallow_vs_deep_profile() {
        let mut shallow = O2ToxUpdateRateAlgorithm::default();
        let mut deep = O2ToxUpdateRateAlgorithm::default();

        let shallow_depths = [
            8.001, 8.002, 8.003, 8.004, 8.005, 8.006, 8.007, 8.008, 8.009, 8.010,
        ];
        let deep_depths = [70.8, 71.6, 72.4, 73.2, 74.0, 74.8, 75.6, 76.4, 77.2, 78.0];

        let mut prev_shallow = msw::new(8.0);
        let mut prev_deep = msw::new(70.0);
        let shallow_rates: Vec<u32> = shallow_depths
            .iter()
            .map(|d| {
                let cur = msw::new(*d);
                let r = shallow.next_iter(prev_shallow, cur).unwrap();
                prev_shallow = cur;
                r
            })
            .collect();
        let deep_rates: Vec<u32> = deep_depths
            .iter()
            .map(|d| {
                let cur = msw::new(*d);
                let r = deep.next_iter(prev_deep, cur).unwrap();
                prev_deep = cur;
                r
            })
            .collect();

        println!("o2 shallow rates: {:?}", shallow_rates);
        println!("o2 deep rates: {:?}", deep_rates);
        let shallow_avg = shallow_rates.iter().sum::<u32>() / shallow_rates.len() as u32;
        let deep_avg = deep_rates.iter().sum::<u32>() / deep_rates.len() as u32;
        assert!(deep_avg < shallow_avg);
    }
}
