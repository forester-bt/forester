use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::blackboard::{BBKey, BlackBoard};
use crate::runtime::context::TreeContext;
use crate::runtime::{RuntimeError, TickResult};

/// Save current tick to bb
pub struct StoreTick;

impl Impl for StoreTick {
    fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick {
        let curr_tick = ctx.curr_ts();
        match args.first() {
            None => Ok(TickResult::Failure(format!(
                "the store_tick has at least one parameter"
            ))),
            Some(v) => {
                let k = v.clone().cast(ctx.bb()).string()?;
                match k {
                    None => Ok(TickResult::Failure(format!("the {v} is not a string",))),
                    Some(key) => ctx
                        .bb()
                        .put(key, RtValue::int(curr_tick as i64))
                        .map(|_| TickResult::success()),
                }
            }
        }
    }
}

/// Compare a value in the cell with the given expected value
pub struct CheckEq;

impl Impl for CheckEq {
    fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick {
        let key = args
            .find_or_ith("key".to_string(), 0)
            .ok_or(RuntimeError::uex(format!("the key is expected ")))?;
        let expected = args
            .find_or_ith("expected".to_string(), 1)
            .ok_or(RuntimeError::uex(format!("the key is expected")))?;

        match key.clone().cast(ctx.bb()).string()? {
            None => Err(RuntimeError::uex(format!("the {key} should be string"))),
            Some(k) => match ctx.bb().get(k.clone())? {
                None => Ok(TickResult::failure(
                    format!("the {key} is not found in bb",),
                )),
                Some(actual) if actual == &expected => Ok(TickResult::success()),
                Some(actual) => Ok(TickResult::failure(format!("{actual} != {expected}"))),
            },
        }
    }
}

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
