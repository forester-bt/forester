use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::blackboard::{BBKey, BlackBoard};
use crate::runtime::context::TreeContext;
use crate::runtime::{RuntimeError, TickResult};

pub struct Fail;

impl Impl for Fail {
    fn tick(&self, args: RtArgs, _ctx: &mut TreeContext) -> Tick {
        let c = args
            .first_as(RtValue::as_string)
            .unwrap_or("fail".to_string());
        Ok(TickResult::failure(c))
    }
}

pub struct GenerateData<T>
where
    T: Fn(RtValue) -> RtValue,
{
    key: BBKey,
    generator: T,
    default: RtValue,
}

impl<T> GenerateData<T>
where
    T: Fn(RtValue) -> RtValue,
{
    pub fn new(key: BBKey, default: RtValue, generator: T) -> Self {
        Self {
            key,
            generator,
            default,
        }
    }
}

impl<T> Impl for GenerateData<T>
where
    T: Fn(RtValue) -> RtValue,
{
    fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick {
        let mut bb: &mut BlackBoard = ctx.bb();
        let curr = bb.get(self.key.clone())?.unwrap_or(&self.default).clone();
        bb.put(self.key.clone(), (self.generator)(curr))?;
        Ok(TickResult::Success)
    }
}
