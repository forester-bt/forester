//! Builtin actions that are related to the data manipulation.
//! The actions are:
//! - `locked` - check if the key is locked in bb
//! - `lock` - lock the key in bb
//! - `unlock` - unlock the key in bb
//! - `store_tick` - save current tick to bb
//! - `check_eq` - compare a value in the cell with the given expected value
//! - `test_bool` - compare a value in the cell with the true
//! - `generate_data` - a simple action that can generate and then update data in the given cell in bb.

use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::TreeContextRef;
use crate::runtime::{RuntimeError, TickResult};

/// Check if the key is locked in BlackBoard
pub struct Locked;

/// Check if the key is locked in BlackBoard
/// Just simple wrapper around the bb api.
/// The key is expected to be a string or a pointer to a string.
impl Impl for Locked {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        ctx.bb().lock()?.is_locked(get_name(args, &ctx)?).map(|v| {
            if v {
                TickResult::success()
            } else {
                TickResult::failure(format!("the key is not locked"))
            }
        })
    }
}

fn get_name(args: RtArgs, ctx: &TreeContextRef) -> Result<String, RuntimeError> {
    args.first()
        .ok_or(RuntimeError::fail(
            "the key argument is not found".to_string(),
        ))
        .and_then(|v| v.cast(ctx.clone()).str())
        .and_then(|v| {
            v.ok_or(RuntimeError::fail(
                "the key argument is not found".to_string(),
            ))
        })
}

/// Lock or unlock key in bb
/// Just simple wrapper around the bb api.
/// The key is expected to be a string or a pointer to a string.
pub enum LockUnlockBBKey {
    Lock,
    Unlock,
}

impl Impl for LockUnlockBBKey {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let key = get_name(args, &ctx)?;

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
        let v = args.first().ok_or(RuntimeError::fail(
            "the store_tick has at least one parameter".to_string(),
        ))?;

        let k = v.clone().cast(ctx.clone()).str()?;
        match k {
            None => Ok(TickResult::failure(format!("the {v} is not a string", ))),
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

/// Compare a value in the cell with the given expected value
impl Impl for CheckEq {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let key = args
            .find_or_ith("key".to_string(), 0)
            .ok_or(RuntimeError::fail("the key is expected ".to_string()))?;

        let expected = args
            .find_or_ith("expected".to_string(), 1)
            .ok_or(RuntimeError::fail("the key is expected".to_string()))?;

        let actual = key.cast(ctx).with_ptr()?;
        if actual == expected {
            Ok(TickResult::success())
        } else {
            Ok(TickResult::failure(format!("{actual} != {expected}")))
        }
    }
}

/// Compare a value in the cell with the true
pub struct TestBool;

impl Impl for TestBool {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {

        let actual = args
            .find_or_ith("key".to_string(), 0)
            .ok_or(RuntimeError::fail("the key is expected ".to_string()))?
            .cast(ctx.clone()).bool()?
            .ok_or(RuntimeError::fail("the key is expected to be a bool".to_string()))?;

        if actual {
            Ok(TickResult::success())
        } else {
            Ok(TickResult::failure(format!("{actual} != true")))
        }
    }
}

/// A simple action that can generate and then update data in the given cell in bb.
/// Encompasses a function that accepts a current value of the cell and then place the updated one.
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
        T: Fn(RtValue) -> RtValue + Send + Sync,
{
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let key = args
            .find_or_ith("key".to_string(), 0)
            .ok_or(RuntimeError::fail(
                "the key is expected and should be a string".to_string(),
            ))?;

        let key = key.cast(ctx.clone()).str()?.ok_or(RuntimeError::fail(
            "the key is expected and should be a string".to_string(),
        ))?;

        let default = args
            .find_or_ith("default".to_string(), 1)
            .ok_or(RuntimeError::fail("the default is expected".to_string()))?;

        let arc_bb = ctx.bb();
        let mut bb = arc_bb.lock()?;
        let curr = bb.get(key.clone())?.unwrap_or(&default).clone();
        bb.put(key, (self.generator)(curr))?;
        Ok(TickResult::Success)
    }
}


pub struct Less;

impl Impl for Less {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let err = |v:&str|
            RuntimeError::fail(v.to_string());

        let lhs =
            args
                .find_or_ith("lhs".to_string(), 0)
                .ok_or(err("lhs should be presented"))?
                .cast(ctx.clone())
                .int()?
                .ok_or(err("the type of lhs is expected as int"))?;
        let rhs =
            args
                .find_or_ith("rhs".to_string(), 1)
                .ok_or(err("rhs should be presented"))?
                .cast(ctx.clone())
                .int()?
                .ok_or(err("the type of rhs is expected as int"))?;

        if lhs < rhs {
            Ok(TickResult::success())
        } else {
            Ok(TickResult::failure_empty())
        }
    }
}

/// Just stores the data to the given cell in bb
pub struct StoreData;

impl Impl for StoreData {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let key = args
            .find_or_ith("key".to_string(), 0)
            .ok_or(RuntimeError::fail(
                "the key is expected and should be a string".to_string(),
            ))?;

        let key = key.cast(ctx.clone()).str()?.ok_or(RuntimeError::fail(
            "the key is expected and should be a string".to_string(),
        ))?;

        let value = args
            .find_or_ith("value".to_string(), 1)
            .ok_or(RuntimeError::fail("the value is expected".to_string()))
            .and_then(|v|v.with_ptr(ctx.clone()))?;

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
    use crate::runtime::context::{TreeContextRef};
    use crate::runtime::trimmer::TrimmingQueue;
    use crate::runtime::{RuntimeError, TickResult};
    use crate::tracer::Tracer;
    
    
    use std::sync::{Arc, Mutex};
    use crate::runtime::env::RtEnv;

    #[test]
    fn lock_unlock() {
        let lock_action = LockUnlockBBKey::Lock;

        let r = lock_action.tick(
            RtArgs(vec![RtArgument::new(
                "key".to_string(),
                RtValue::str("k".to_string()),
            )]),
            TreeContextRef::new(
                Arc::new(Mutex::new(BlackBoard::default())),
                Arc::new(Mutex::new(Tracer::Noop)),
                1,
                Arc::new(Mutex::new(TrimmingQueue::default())),
                Arc::new(Mutex::new(RtEnv::try_new().unwrap())),
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
            TreeContextRef::new(
                bb.clone(),
                Arc::new(Mutex::new(Tracer::Noop)),
                1,
                Arc::new(Mutex::new(TrimmingQueue::default())),
                Arc::new(Mutex::new(RtEnv::try_new().unwrap())),
            ),
        );
        assert_eq!(r, Ok(TickResult::success()));
        assert_eq!(
            bb.clone().lock().unwrap().is_locked("k".to_string()),
            Ok(true)
        );
    }

    #[test]
    fn store_tick() {
        let store_tick = super::StoreTick;

        let bb = Arc::new(Mutex::new(BlackBoard::default()));

        let r = store_tick.tick(
            RtArgs(vec![RtArgument::new_noname(RtValue::str("k".to_string()))]),
            TreeContextRef::new(
                bb.clone(),
                Arc::new(Mutex::new(Tracer::Noop)),
                1,
                Arc::new(Mutex::new(TrimmingQueue::default())),
                Arc::new(Mutex::new(RtEnv::try_new().unwrap())),
            ),
        );
        assert_eq!(r, Ok(TickResult::success()));
        assert_eq!(
            bb.clone().lock().unwrap().get("k".to_string()),
            Ok(Some(&RtValue::int(1)))
        );
    }
}
