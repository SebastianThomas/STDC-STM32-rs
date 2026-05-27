use core::fmt::Display;

use libm::sinf;
use stdc_diving_algorithms::pressure_unit::{Pa, Pressure, msw};
const DECO_TRANSIT_ASCENT_RATE_M_PER_MIN: f32 = 9.0;
// use crate::components::display::MAX_STOP_NUMS;
const MAX_STOP_NUMS: usize = 31;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DiveStage {
    Descent,
    Bottom,
    DecoHold,
    Ascent,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EmulationDecoStop {
    pub stop_depth_pa: Pa,
    pub hold_samples: usize,
    pub start_sample: Option<usize>,
}

impl EmulationDecoStop {
    pub const fn none() -> Self {
        EmulationDecoStop {
            stop_depth_pa: msw(0.0).to_pa(),
            hold_samples: 0,
            start_sample: None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EmulationDecoOverlay {
    pub stops: [EmulationDecoStop; MAX_STOP_NUMS],
    pub count: usize,
}

impl Display for EmulationDecoOverlay {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_fmt(format_args!("DecoOverlay: count={} ", self.count))?;
        for i in 0..self.count.min(MAX_STOP_NUMS) {
            let s = &self.stops[i];
            f.write_fmt(format_args!("[{}: depth={:?} samples={} start={:?}] ", i, s.stop_depth_pa.to_msw(), s.hold_samples, s.start_sample))?;
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EmulatedDiveProfile {
    stage_lengths: [usize; 3],
    max_depth_delta_pa: Pa,
    bottom_variation: f32,
    sample_interval_ms: u32,
    deco_overlay: Option<EmulationDecoOverlay>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EmulatedDivePoint {
    pub sample_index: usize,
    pub time_ms: u32,
    pub stage: DiveStage,
    pub depth_pa: Pa,
    pub depth_msw: msw,
}

impl EmulatedDiveProfile {
    pub const fn new(
        stage_lengths: [usize; 3],
        max_depth_delta_pa: Pa,
        bottom_variation: f32,
    ) -> Self {
        Self {
            stage_lengths,
            max_depth_delta_pa,
            bottom_variation,
            sample_interval_ms: 1_000,
            deco_overlay: None,
        }
    }

    pub const fn small() -> Self {
        Self::new([32, 64, 32], msw::new(25.0).to_pa(), 0.05)
    }

    pub const fn large() -> Self {
        Self::new([48, 96, 48], msw::new(25.0).to_pa(), 0.04)
    }

    pub const fn deep() -> Self {
        Self::new([40, 320, 160], msw::new(90.0).to_pa(), 0.02)
    }

    /// Create an emulated profile based on physical rates and a time step.
    ///
    /// - `time_step_ms` is the sample interval in milliseconds (live-sim uses 1000).
    /// - `descent_m_per_min` default 20.0 m/min (slower descent).
    /// - `bottom_minutes` default 10.0 minutes (fixed bottom hold).
    /// Ascent is computed piecewise: 18 m/min until half max depth, then 9 m/min,
    /// and for depths <= 21 m the ascent portion uses 3 m/min for a conservative profile.
    pub fn with_rates(
        max_depth_delta_pa: Pa,
        time_step_ms: u32,
        descent_m_per_min: f32,
        bottom_minutes: f32,
    ) -> Self {
        let max_depth_m = max_depth_delta_pa.to_msw().to_f32();

        // Descent samples
        let descent_secs = if descent_m_per_min > 0.0 {
            (max_depth_m / descent_m_per_min) * 60.0
        } else {
            0.0
        };
        let descent_samples = libm::ceilf((descent_secs * 1000.0) / time_step_ms as f32) as usize;

        let bottom_secs = (bottom_minutes * 60.0).max(0.0);
        let bottom_samples = libm::ceilf((bottom_secs * 1000.0) / time_step_ms as f32) as usize;

        let half_depth = max_depth_m / 2.0;
        let mut remaining_depth = max_depth_m;
        let mut ascent_secs = 0.0f32;

        if remaining_depth > half_depth {
            let seg = remaining_depth - half_depth;
            ascent_secs += (seg / 18.0) * 60.0;
            remaining_depth = half_depth;
        }

        if remaining_depth > 21.0 {
            let seg = remaining_depth - 21.0;
            ascent_secs += (seg / 9.0) * 60.0;
            remaining_depth = 21.0;
        }

        if remaining_depth > 0.0 {
            ascent_secs += (remaining_depth / 3.0) * 60.0;
        }

        let ascent_samples = libm::ceilf((ascent_secs * 1000.0) / time_step_ms as f32) as usize;

        Self::new(
            [descent_samples, bottom_samples, ascent_samples],
            max_depth_delta_pa,
            0.02,
        )
        .with_sample_interval_ms(time_step_ms)
    }

    pub fn deep_with_defaults(time_step_ms: u32) -> Self {
        Self::with_rates(msw::new(90.0).to_pa(), time_step_ms, 20.0, 10.0)
    }

    pub fn deco_start_sample(&self) -> usize {
        self.stage_lengths[0] + self.stage_lengths[1]
    }

    pub fn max_depth_delta_pa(&self) -> Pa {
        self.max_depth_delta_pa
    }

    pub const fn with_sample_interval_ms(mut self, sample_interval_ms: u32) -> Self {
        self.sample_interval_ms = if sample_interval_ms == 0 {
            1
        } else {
            sample_interval_ms
        };
        self
    }

    fn deco_overlay_uses_absolute_starts(overlay: &EmulationDecoOverlay) -> bool {
        for i in 0..overlay.count.min(MAX_STOP_NUMS) {
            if overlay.stops[i].start_sample.is_some() {
                return true;
            }
        }
        false
    }

    fn deco_overlay_duration_samples(&self, overlay: &EmulationDecoOverlay) -> usize {
        let deco_start = self.deco_start_sample();

        if Self::deco_overlay_uses_absolute_starts(overlay) {
            let mut previous_end = deco_start;
            let mut deco_end = deco_start;
            for i in 0..overlay.count.min(MAX_STOP_NUMS) {
                let stop = &overlay.stops[i];
                let start = stop.start_sample.unwrap_or(previous_end);
                let end = start.saturating_add(stop.hold_samples);
                deco_end = deco_end.max(end);
                previous_end = end;
            }
            deco_end.saturating_sub(deco_start)
        } else {
            let mut previous_depth_m = self.max_depth_delta_pa.to_msw().to_f32();
            let mut hold_samples = 0usize;
            for i in 0..overlay.count.min(MAX_STOP_NUMS) {
                hold_samples += overlay.stops[i].hold_samples;
                let stop_depth_m = overlay.stops[i].stop_depth_pa.to_msw().to_f32();
                let transit_depth_m = (previous_depth_m - stop_depth_m).max(0.0);
                if transit_depth_m > 0.0 {
                    let transit_secs = (transit_depth_m / DECO_TRANSIT_ASCENT_RATE_M_PER_MIN) * 60.0;
                    hold_samples +=
                        libm::ceilf((transit_secs * 1000.0) / self.sample_interval_ms as f32)
                            as usize;
                }
                previous_depth_m = stop_depth_m;
            }
            hold_samples
        }
    }

    pub const fn with_deco_overlay(mut self, stop_depth_pa: Pa, hold_samples: usize) -> Self {
        let mut stops = [EmulationDecoStop::none(); MAX_STOP_NUMS];
        stops[0] = EmulationDecoStop {
            stop_depth_pa,
            hold_samples,
            start_sample: None,
        };
        self.deco_overlay = Some(EmulationDecoOverlay { stops, count: 1 });
        self
    }

    pub fn with_deco_overlay_obj(mut self, overlay: EmulationDecoOverlay) -> Self {
        self.deco_overlay = Some(overlay);
        self
    }

    pub const fn without_deco_overlay(mut self) -> Self {
        self.deco_overlay = None;
        self
    }

    pub fn cycle_length(&self) -> usize {
        let base = self.stage_lengths.iter().sum::<usize>();
        if let Some(overlay) = self.deco_overlay {
            base + self.deco_overlay_duration_samples(&overlay)
        } else {
            base
        }
    }

    fn stage_at(&self, sample_index: usize) -> (DiveStage, f32) {
        let cycle_length = self.cycle_length().max(1);
        if sample_index >= cycle_length {
            return (DiveStage::Ascent, 1.0);
        }
        let cycle_index = sample_index % cycle_length;

        let descent_end = self.stage_lengths[0];
        let bottom_end = descent_end + self.stage_lengths[1];
        let mut deco_hold_end = bottom_end;
        if let Some(overlay) = self.deco_overlay {
            deco_hold_end = bottom_end + self.deco_overlay_duration_samples(&overlay);
        }

        if cycle_index < descent_end {
            (
                DiveStage::Descent,
                cycle_index as f32 / self.stage_lengths[0].max(1) as f32,
            )
        } else if cycle_index < bottom_end {
            (
                DiveStage::Bottom,
                (cycle_index - descent_end) as f32 / self.stage_lengths[1].max(1) as f32,
            )
        } else if cycle_index < deco_hold_end {
            (DiveStage::DecoHold, 0.0)
        } else {
            let ascent_index = cycle_index - deco_hold_end;
            (
                DiveStage::Ascent,
                ascent_index as f32 / self.stage_lengths[2].max(1) as f32,
            )
        }
    }

    pub fn depth_factor_at(&self, sample_index: usize) -> f32 {
        let (stage, stage_progress) = self.stage_at(sample_index);
        match stage {
            DiveStage::Descent => stage_progress,
            DiveStage::Bottom => {
                let wobble = sinf(stage_progress * core::f32::consts::TAU);
                1.0 + self.bottom_variation * wobble
            }
            DiveStage::DecoHold => 1.0,
            DiveStage::Ascent => 1.0 - stage_progress,
        }
    }

    pub fn depth_at(&self, surface_pa: Pa, sample_index: usize) -> Pa {
        let (stage, stage_progress) = self.stage_at(sample_index);
        match stage {
            DiveStage::Descent => surface_pa + self.max_depth_delta_pa * stage_progress,
            DiveStage::Bottom => {
                let wobble = sinf(stage_progress * core::f32::consts::TAU);
                surface_pa + self.max_depth_delta_pa * (self.bottom_variation * wobble + 1.0)
            }
            DiveStage::DecoHold => {
                if let Some(overlay) = self.deco_overlay {
                    let deco_start = self.deco_start_sample();
                    let mut previous_end = deco_start;
                    let mut previous_depth = surface_pa + self.max_depth_delta_pa;
                    for i in 0..overlay.count.min(MAX_STOP_NUMS) {
                        let s = &overlay.stops[i];
                        let start = if let Some(start_sample) = s.start_sample {
                            start_sample
                        } else {
                            let stop_depth_m = s.stop_depth_pa.to_msw().to_f32();
                            let previous_depth_m = previous_depth.to_msw().to_f32();
                            let transit_depth_m = (previous_depth_m - stop_depth_m).max(0.0);
                            let transit_samples = if transit_depth_m > 0.0 {
                                let transit_secs =
                                    (transit_depth_m / DECO_TRANSIT_ASCENT_RATE_M_PER_MIN) * 60.0;
                                libm::ceilf(
                                    (transit_secs * 1000.0)
                                        / self.sample_interval_ms as f32,
                                ) as usize
                            } else {
                                0
                            };
                            previous_end + transit_samples
                        };
                        if sample_index < start {
                            let transit_samples = start.saturating_sub(previous_end).max(1);
                            let progress =
                                (sample_index - previous_end) as f32 / transit_samples as f32;
                            return previous_depth + (s.stop_depth_pa - previous_depth) * progress;
                        }
                        let end = start.saturating_add(s.hold_samples);
                        if sample_index < end {
                            return s.stop_depth_pa;
                        }
                        previous_depth = s.stop_depth_pa;
                        previous_end = end;
                    }
                }
                surface_pa
            }
            DiveStage::Ascent => surface_pa + self.max_depth_delta_pa * (1.0 - stage_progress),
        }
    }

    pub fn point_at(
        &self,
        surface: Pa,
        time_step_ms: u32,
        sample_index: usize,
    ) -> EmulatedDivePoint {
        let (stage, _) = self.stage_at(sample_index);
        let depth_pa = self.depth_at(surface, sample_index);
        EmulatedDivePoint {
            sample_index,
            time_ms: sample_index as u32 * time_step_ms,
            stage,
            depth_pa,
            depth_msw: depth_pa.to_msw(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deco_overlay_interpolates_between_stops() {
        let mut stops = [EmulationDecoStop::none(); MAX_STOP_NUMS];
        stops[0] = EmulationDecoStop {
            stop_depth_pa: msw::new(30.0).to_pa(),
            hold_samples: 2,
            start_sample: Some(5),
        };
        stops[1] = EmulationDecoStop {
            stop_depth_pa: msw::new(20.0).to_pa(),
            hold_samples: 2,
            start_sample: Some(10),
        };

        let profile = EmulatedDiveProfile::new([2, 2, 2], msw::new(60.0).to_pa(), 0.0)
            .with_deco_overlay_obj(EmulationDecoOverlay { stops, count: 2 });

        let surface = Pa::new(101_325.0);
        let before_first_stop = profile.point_at(surface, 1_000, 4).depth_msw.to_f32();
        let first_stop = profile.point_at(surface, 1_000, 5).depth_msw.to_f32();
        let between_stops = profile.point_at(surface, 1_000, 8).depth_msw.to_f32();
        let second_stop = profile.point_at(surface, 1_000, 10).depth_msw.to_f32();

        assert!(before_first_stop > first_stop);
        assert!(first_stop > second_stop);
        assert!(between_stops < first_stop && between_stops > second_stop);
        assert!(second_stop > 0.0);
    }
}
