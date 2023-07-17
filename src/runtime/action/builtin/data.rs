use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::blackboard::{BBKey, BlackBoard};
use crate::runtime::context::{TreeContext, TreeContextRef};
use crate::runtime::{RuntimeError, TickResult};

/// Lock or unlock key in bb
/// Just simple wrapper around the bb api.
pub enum LockUnlockBBKey {
    Lock,
    Unlock,
}
impl Impl for LockUnlockBBKey {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let key = args
            .first()
            .and_then(RtValue::as_string)
            .ok_or(RuntimeError::fail(format!("the key argument is not found")))?;

        match &self {
            LockUnlockBBKey::Lock => ctx.bb().lock()?.lock(key)?,
            LockUnlockBBKey::Unlock => ctx.bb().lock()?.unlock(key)?,
        }
        Ok(TickResult::Success)
    }
}
/// Save current tick to bb
pub struct StoreTick;

impl Impl for StoreTick {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let curr_tick = ctx.current_tick();
        let v = args.first().ok_or(RuntimeError::fail(format!(
            "the store_tick has at least one parameter"
        )))?;

        // let k = v.clone().cast(ctx.bb()).string()?;
        let k = v.clone().as_string();
        match k {
            None => Ok(TickResult::failure(format!("the {v} is not a string",))),
            Some(key) => ctx
                .bb()
                .lock()?
                .put(key, RtValue::int(curr_tick as i64))
                .map(|_| TickResult::success()),
        }
    }
}

/// Compare a value in the cell with the given expected value
pub struct CheckEq;

impl Impl for CheckEq {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let key = args
            .find_or_ith("key".to_string(), 0)
            .ok_or(RuntimeError::fail(format!("the key is expected ")))?;
        let expected = args
            .find_or_ith("expected".to_string(), 1)
            .ok_or(RuntimeError::fail(format!("the key is expected")))?;

        // let option = key.clone().cast(ctx.bb()).string()?;
        let option = key.clone().as_string();
        match option {
            None => Err(RuntimeError::fail(format!("the {key} should be a string"))),
            Some(k) => match ctx.bb().lock()?.get(k.clone())? {
                None => Ok(TickResult::failure(
                    format!("the {key} is not found in bb",),
                )),
                Some(actual) if actual == &expected => Ok(TickResult::success()),
                Some(actual) => Ok(TickResult::failure(format!("{actual} != {expected}"))),
            },
        }
    }
}

/// A simple action that can generate and then update data in the given cell in bb.
/// Encompassess a function that accepts a current value of the cell and then place the updated one.
///
/// ## Note:
/// The action accepts a default parameter that will be used initially.
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
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let key = args
            .find_or_ith("key".to_string(), 0)
            .and_then(|k| k.as_string())
            .ok_or(RuntimeError::fail(format!(
                "the key is expected and should be a string"
            )))?;

        let default = args
            .find_or_ith("default".to_string(), 1)
            .ok_or(RuntimeError::fail(format!("the default is expected")))?;

        let arc_bb = ctx.bb();
        let mut bb = arc_bb.lock()?;
        let curr = bb.get(key.clone())?.unwrap_or(&default).clone();
        bb.put(key, (self.generator)(curr))?;
        Ok(TickResult::Success)
    }
}
/// Just stores the data to the given cell in bb
pub struct StoreData;

impl Impl for StoreData {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let key = args
            .find_or_ith("key".to_string(), 0)
            .and_then(|k| k.as_string())
            .ok_or(RuntimeError::fail(format!(
                "the key is expected and should be a string"
            )))?;

        let value = args
            .find_or_ith("value".to_string(), 1)
            .ok_or(RuntimeError::fail(format!("the value is expected")))?;

        ctx.bb().lock()?.put(key, value)?;
        Ok(TickResult::Success)
    }
}

#[cfg(test)]
mod tests {
    use crate::runtime::action::builtin::data::LockUnlockBBKey;
    use crate::runtime::action::Impl;
    use crate::runtime::args::{RtArgs, RtArgument, RtValue};
    use crate::runtime::blackboard::{BBValue, BlackBoard};
    use crate::runtime::context::{TreeContext, TreeContextRef};
    use crate::runtime::{RuntimeError, TickResult};
    use crate::tracer::Tracer;
    use log::Level::Trace;
    use std::collections::HashMap;
    use std::sync::{Arc, Mutex};

    #[test]
    fn lock_unlock() {
        let mut lock_action = LockUnlockBBKey::Lock;

        let r = lock_action.tick(
            RtArgs(vec![RtArgument::new(
                "key".to_string(),
                RtValue::str("k".to_string()),
            )]),
            TreeContextRef::new(
                Arc::new(Mutex::new(BlackBoard::default())),
                Arc::new(Mutex::new(Tracer::Noop)),
                1,
            ),
        );
        assert_eq!(
            r,
            Err(RuntimeError::BlackBoardError(
                "the key k is taken or absent".to_string()
            ))
        );

        let bb = Arc::new(Mutex::new(BlackBoard::new(vec![(
            "k".to_string(),
            BBValue::Unlocked(RtValue::int(1)),
        )])));
        let r = lock_action.tick(
            RtArgs(vec![RtArgument::new(
                "key".to_string(),
                RtValue::str("k".to_string()),
            )]),
            TreeContextRef::new(bb.clone(), Arc::new(Mutex::new(Tracer::Noop)), 1),
        );
        assert_eq!(r, Ok(TickResult::success()));
        assert_eq!(
            bb.clone().lock().unwrap().is_locked("k".to_string()),
            Ok(true)
        );
    }
}
