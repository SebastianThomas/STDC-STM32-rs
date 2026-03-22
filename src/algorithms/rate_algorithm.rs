use core::{
    num::NonZeroU32,
    ops::{Add, Div, Mul, Sub},
};

const NON_ZERO_2: NonZeroU32 = NonZeroU32::new(2).unwrap();

pub trait RateAlgorithm<P, I, Output> {
    type Error;
    fn next_iter(&mut self, prev: P, instance: I) -> Result<Output, Self::Error>;
}

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

pub trait AimdUnit:
    Add<Output = Self>
    + Sub<Output = Self>
    + Mul<u32, Output = Self>
    + Div<NonZeroU32, Output = Self>
    + Ord
    + Copy
{
}

impl AimdUnit for u32 {}

pub struct AimdRateAlgorithm<Unit: AimdUnit> {
    current: Unit,
    max: Unit,
    ai_millis: Unit,
    md: NonZeroU32,
}

impl<Unit: AimdUnit> AimdRateAlgorithm<Unit> {
    pub fn new(
        initial: Unit,
        max: Unit,
        ai_millis: Unit,
        md_multiplicator: NonZeroU32,
    ) -> AimdRateAlgorithm<Unit> {
        AimdRateAlgorithm {
            current: initial.min(max),
            max,
            ai_millis,
            md: md_multiplicator,
        }
    }
}

impl<Unit: AimdUnit, P> RateAlgorithm<P, bool, Unit> for AimdRateAlgorithm<Unit> {
    type Error = ();
    fn next_iter(&mut self, _prev: P, change: bool) -> Result<Unit, Self::Error> {
        if change {
            self.current = self.current / self.md
        } else {
            self.current = self.current + self.ai_millis
        }
        self.current = self.current.min(self.max);
        Ok(self.current)
    }
}

pub struct DiffAimdRateAlgorithm<ResultUnit: AimdUnit, Unit: AimdUnit> {
    base_algorithm: AimdRateAlgorithm<ResultUnit>,
    min_change_diff: Unit,
}

impl<ResultUnit: AimdUnit, Unit: AimdUnit> DiffAimdRateAlgorithm<ResultUnit, Unit> {
    pub fn new(
        initial: ResultUnit,
        max: ResultUnit,
        min_change_diff: Unit,
    ) -> DiffAimdRateAlgorithm<ResultUnit, Unit> {
        DiffAimdRateAlgorithm {
            base_algorithm: AimdRateAlgorithm::new(initial, max, initial, NON_ZERO_2),
            min_change_diff,
        }
    }
}

impl<ResultUnit: AimdUnit, Unit: AimdUnit> RateAlgorithm<Unit, Unit, ResultUnit>
    for DiffAimdRateAlgorithm<ResultUnit, Unit>
{
    type Error = ();
    fn next_iter(&mut self, prev: Unit, instance: Unit) -> Result<ResultUnit, Self::Error> {
        let diff = if prev > instance {
            prev - instance
        } else {
            instance - prev
        };
        let change = diff >= self.min_change_diff;
        self.base_algorithm.next_iter((), change)
    }
}

pub struct DynamicDiffAimdRateAlgorithm<Unit: AimdUnit> {
    base_algorithm: DiffAimdRateAlgorithm<u32, Unit>,
    diff_algorithm: DiffAimdRateAlgorithm<Unit, Unit>,
}

impl<Unit: AimdUnit> DynamicDiffAimdRateAlgorithm<Unit> {
    pub fn new(
        initial: u32,
        max: u32,
        initial_min_diff: Unit,
        max_diff: Unit,
        min_diff_change_diff: Unit,
    ) -> DynamicDiffAimdRateAlgorithm<Unit> {
        DynamicDiffAimdRateAlgorithm {
            base_algorithm: DiffAimdRateAlgorithm::new(initial, max, initial_min_diff),
            diff_algorithm: DiffAimdRateAlgorithm::new(
                initial_min_diff,
                max_diff,
                min_diff_change_diff,
            ),
        }
    }
}

impl<Unit: AimdUnit> RateAlgorithm<Unit, Unit, u32> for DynamicDiffAimdRateAlgorithm<Unit> {
    type Error = ();
    fn next_iter(&mut self, prev: Unit, instance: Unit) -> Result<u32, Self::Error> {
        let diff_res = self.diff_algorithm.next_iter(prev, instance);
        if let Ok(diff) = diff_res {
            self.base_algorithm.min_change_diff = diff;
        }
        self.base_algorithm.next_iter(prev, instance)
    }
}
