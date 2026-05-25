use core::fmt::Display;

use libm::sinf;
use stdc_diving_algorithms::pressure_unit::{Pa, Pressure, msw};
use crate::components::display::MAX_STOP_NUMS;

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
    }

    pub fn deep_with_defaults(time_step_ms: u32) -> Self {
        Self::with_rates(msw::new(90.0).to_pa(), time_step_ms, 20.0, 10.0)
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
        let overlay_samples = if let Some(overlay) = self.deco_overlay {
            let mut s = 0usize;
            for i in 0..overlay.count.min(MAX_STOP_NUMS) {
                s += overlay.stops[i].hold_samples;
            }
            s
        } else {
            0
        };
        base + overlay_samples
    }

    fn stage_at(&self, sample_index: usize) -> (DiveStage, f32) {
        let cycle_length = self.cycle_length().max(1);
        if sample_index >= cycle_length {
            return (DiveStage::Ascent, 1.0);
        }
        let cycle_index = sample_index % cycle_length;

        let descent_end = self.stage_lengths[0];
        let bottom_end = descent_end + self.stage_lengths[1];
        // Determine deco hold segments. If overlay is present, use its
        // configured stops and their optional start_sample; otherwise default
        // to a single hold immediately after bottom.
        let deco_hold_start: usize;
        let mut deco_hold_end = bottom_end;
        if let Some(overlay) = self.deco_overlay {
            // If any stop has a start_sample set, we assume the overlay
            // defines absolute positions. Otherwise we place the stops
            // sequentially starting after bottom.
            let mut any_start = false;
            for i in 0..overlay.count.min(MAX_STOP_NUMS) {
                if overlay.stops[i].start_sample.is_some() {
                    any_start = true;
                    break;
                }
            }
            if any_start {
                // Find the minimum start_sample to define the hold region start.
                let mut min_start = usize::MAX;
                for i in 0..overlay.count.min(MAX_STOP_NUMS) {
                    if let Some(s) = overlay.stops[i].start_sample {
                        if s < min_start {
                            min_start = s;
                        }
                    }
                }
                if min_start != usize::MAX {
                    deco_hold_start = min_start;
                    // compute end as max(start + hold) across stops
                    let mut max_end = deco_hold_start;
                    for i in 0..overlay.count.min(MAX_STOP_NUMS) {
                        let end = overlay.stops[i].start_sample.unwrap_or(0) + overlay.stops[i].hold_samples;
                        if end > max_end {
                            max_end = end;
                        }
                    }
                    deco_hold_end = max_end;
                }
            } else {
                // Place stops sequentially after bottom_end
                let mut acc = bottom_end;
                for i in 0..overlay.count.min(MAX_STOP_NUMS) {
                    acc += overlay.stops[i].hold_samples;
                }
                deco_hold_end = acc;
            }
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
                    // Determine which stop we are in by walking stops in order.
                    let mut idx = None;
                    let mut pos = self.stage_lengths.iter().sum::<usize>();
                    for i in 0..overlay.count.min(MAX_STOP_NUMS) {
                        let s = &overlay.stops[i];
                        if sample_index >= pos && sample_index < pos + s.hold_samples {
                            idx = Some(i);
                            break;
                        }
                        pos += s.hold_samples;
                    }
                    if let Some(i) = idx {
                        return overlay.stops[i].stop_depth_pa;
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
