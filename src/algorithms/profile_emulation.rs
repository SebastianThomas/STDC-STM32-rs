use libm::sinf;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DiveStage {
    Descent,
    Bottom,
    DecoHold,
    Ascent,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EmulationDecoOverlay {
    pub stop_depth_pa: f32,
    pub hold_samples: usize,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EmulatedDiveProfile {
    stage_lengths: [usize; 3],
    max_depth_delta_pa: f32,
    bottom_variation: f32,
    deco_overlay: Option<EmulationDecoOverlay>,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EmulatedDivePoint {
    pub sample_index: usize,
    pub time_ms: u32,
    pub stage: DiveStage,
    pub depth_pa: f32,
    pub depth_msw: f32,
}

impl EmulatedDiveProfile {
    pub const fn new(
        stage_lengths: [usize; 3],
        max_depth_delta_pa: f32,
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
        Self::new([32, 64, 32], 250_000.0, 0.05)
    }

    pub const fn large() -> Self {
        Self::new([48, 96, 48], 250_000.0, 0.04)
    }

    pub const fn deep() -> Self {
        Self::new([40, 320, 160], 9.0 * 100_000.0, 0.02)
    }

    pub const fn with_deco_overlay(mut self, stop_depth_pa: f32, hold_samples: usize) -> Self {
        self.deco_overlay = Some(EmulationDecoOverlay {
            stop_depth_pa,
            hold_samples,
        });
        self
    }

    pub const fn without_deco_overlay(mut self) -> Self {
        self.deco_overlay = None;
        self
    }

    pub fn cycle_length(&self) -> usize {
        self.stage_lengths.iter().sum::<usize>()
            + self
                .deco_overlay
                .map(|overlay| overlay.hold_samples)
                .unwrap_or(0)
    }

    fn stage_at(&self, sample_index: usize) -> (DiveStage, f32) {
        let cycle_length = self.cycle_length().max(1);
        let cycle_index = sample_index % cycle_length;

        let descent_end = self.stage_lengths[0];
        let bottom_end = descent_end + self.stage_lengths[1];
        let deco_hold_end = bottom_end
            + self
                .deco_overlay
                .map(|overlay| overlay.hold_samples)
                .unwrap_or(0);

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

    pub fn depth_at(&self, surface_pa: f32, sample_index: usize) -> f32 {
        let (stage, stage_progress) = self.stage_at(sample_index);
        match stage {
            DiveStage::Descent => surface_pa + stage_progress * self.max_depth_delta_pa,
            DiveStage::Bottom => {
                let wobble = sinf(stage_progress * core::f32::consts::TAU);
                surface_pa + (1.0 + self.bottom_variation * wobble) * self.max_depth_delta_pa
            }
            DiveStage::DecoHold => self
                .deco_overlay
                .map(|overlay| overlay.stop_depth_pa)
                .unwrap_or(surface_pa),
            DiveStage::Ascent => surface_pa + (1.0 - stage_progress) * self.max_depth_delta_pa,
        }
    }

    pub fn point_at(
        &self,
        surface_pa: f32,
        time_step_ms: u32,
        sample_index: usize,
    ) -> EmulatedDivePoint {
        let (stage, _) = self.stage_at(sample_index);
        let depth_pa = self.depth_at(surface_pa, sample_index);
        EmulatedDivePoint {
            sample_index,
            time_ms: sample_index as u32 * time_step_ms,
            stage,
            depth_pa,
            depth_msw: depth_pa / 100_000.0,
        }
    }
}
