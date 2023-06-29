use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::blackboard::{BBKey, BlackBoard};
use crate::runtime::context::TreeContext;
use crate::runtime::{RuntimeError, TickResult};

pub struct GenerateData<T>
where
    T: Fn(RtValue) -> RtValue,
{
    generator: T,
}

impl<T> GenerateData<T>
where
    T: Fn(RtValue) -> RtValue,
{
    pub fn new(generator: T) -> Self {
        Self { generator }
    }
}

impl<T> Impl for GenerateData<T>
where
    T: Fn(RtValue) -> RtValue,
{
    fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick {
        let key = args
            .find_or_ith("key".to_string(), 0)
            .and_then(|k| k.as_string())
            .ok_or(RuntimeError::uex(format!(
                "the key is expected and should be a string"
            )))?;

        let default = args
            .find_or_ith("default".to_string(), 1)
            .ok_or(RuntimeError::uex(format!("the default is expected")))?;

        let mut bb: &mut BlackBoard = ctx.bb();
        let curr = bb.get(key.clone())?.unwrap_or(&default).clone();
        bb.put(key, (self.generator)(curr))?;
        Ok(TickResult::Success)
    }
}

pub struct StoreData;

impl Impl for StoreData {
    fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick {
        let key = args
            .find_or_ith("key".to_string(), 0)
            .and_then(|k| k.as_string())
            .ok_or(RuntimeError::uex(format!(
                "the key is expected and should be a string"
            )))?;

        let value = args
            .find_or_ith("value".to_string(), 1)
            .ok_or(RuntimeError::uex(format!("the value is expected")))?;

        ctx.bb().put(key, value)?;
        Ok(TickResult::Success)
    }
}
