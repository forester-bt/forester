pub mod builtin;
pub mod decorator;
pub mod flow;
pub mod keeper;

use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::TreeContext;
use crate::runtime::{RtResult, RuntimeError, TickResult};
use std::collections::HashMap;

pub type ActionName = String;
pub type Tick = RtResult<TickResult>;

/// Recovers the tick depending on the result.
fn recover(tick: Tick) -> Tick {
    match tick {
        Err(RuntimeError::RecoveryToFailure(r)) => Ok(TickResult::Failure(r)),
        Err(RuntimeError::BlackBoardError(r)) => Ok(TickResult::Failure(r)),
        other => other,
    }
}

/// The Action wrapper that provides two implementations:
/// - sync that is used by default
/// - astnc to handle the future (uses tokio under the hood)
pub enum Action {
    Impl(Box<dyn Impl>),
    Async(Box<dyn ImplAsync>),
}

impl Action {
    pub fn sync<T>(a: T) -> Self
    where
        T: Impl + 'static,
    {
        Action::Impl(Box::new(a))
    }

    pub fn asynch<T>(a: T) -> Self
    where
        T: ImplAsync + 'static,
    {
        Action::Async(Box::new(a))
    }
}

impl Action {
    pub fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick {
        recover(match self {
            Action::Impl(a) => a.tick(args, ctx),
            Action::Async(aa) => aa.tick(args, ctx),
        })
    }
}

/// The main trait to implement stateless sync action
/// # Params
/// - args: the list of arguments that will be accepted from the given operation from the forester
/// - ctx: the tree runtime context
///
/// ## Returns tick = RtResult(TickResult)
///
/// # Notes
/// The simple and blocking implementation so better off to avoid using it with heavy actions.
/// The mutability is disabled, thus to keep some parameters better off to use bb or async action.
///
/// Also, even giving the possibility to return `TickResult::running()` better to avoid it in favor
/// to using the async action.
///
/// # Example
/// ```rust
///
/// use crate::runtime::action::{recover, Impl, Tick};
/// use crate::runtime::args::{RtArgs, RtValue};
/// use crate::runtime::blackboard::{BBKey, BlackBoard};
/// use crate::runtime::context::TreeContext;
/// use crate::runtime::{RuntimeError, TickResult};
///
/// pub struct GenerateData<T>
/// where
///     T: Fn(RtValue) -> RtValue,
/// {
///     generator: T,
/// }
///
/// impl<T> GenerateData<T>
/// where
///     T: Fn(RtValue) -> RtValue,
/// {
///     pub fn new(generator: T) -> Self {
///         Self { generator }
///     }
/// }
///
/// impl<T> Impl for GenerateData<T>
/// where
///     T: Fn(RtValue) -> RtValue,
/// {
///     fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick {
///         let key = args
///             .find_or_ith("key".to_string(), 0)
///             .and_then(|k| k.as_string())
///             .ok_or(RuntimeError::fail(format!(
///                 "the key is expected and should be a string"
///             )))?;
///
///         let default = args
///             .find_or_ith("default".to_string(), 1)
///             .ok_or(RuntimeError::fail(format!("the default is expected")))?;
///
///         let mut bb: &mut BlackBoard = ctx.bb();
///         let curr = bb.get(key.clone())?.unwrap_or(&default).clone();
///         bb.put(key, (self.generator)(curr))?;
///         Ok(TickResult::Success)
///     }
/// }
/// ```
pub trait Impl {
    fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick;
}

pub trait ImplAsync {
    async fn tick(&mut self, args: RtArgs, ctx: &mut TreeContext) -> Tick;
}

impl From<Box<dyn Impl>> for Action {
    fn from(value: Box<dyn Impl>) -> Self {
        Action::Impl(value)
    }
}

impl From<Box<dyn ImplAsync>> for Action {
    fn from(value: Box<dyn ImplAsync>) -> Self {
        Action::Async(value)
    }
}
