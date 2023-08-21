pub mod builtin;
pub mod keeper;

use crate::runtime::args::RtArgs;
use crate::runtime::context::{TreeContextRef, TreeRemoteContextRef};
use crate::runtime::{RtResult, RuntimeError, TickResult};
use std::sync::Arc;

pub type ActionName = String;
pub type Tick = RtResult<TickResult>;

/// Recovers the tick depending on the result.
pub fn recover(tick: Tick) -> Tick {
    match tick {
        Err(RuntimeError::RecoveryToFailure(r)) => Ok(TickResult::Failure(r)),
        Err(RuntimeError::BlackBoardError(r)) => Ok(TickResult::Failure(r)),
        other => other,
    }
}

/// The Action wrapper that provides two implementations:
/// - sync that is used by default
/// - async to handle the future (uses tokio under the hood)
/// - remote to handle the remote actions
pub enum Action {
    Sync(Box<dyn Impl>),
    Async(Arc<dyn ImplAsync>),
    Remote(Box<dyn ImplRemote>),
}

impl Action {
    pub fn sync<T>(a: T) -> Self
    where
        T: Impl + 'static,
    {
        Action::Sync(Box::new(a))
    }

    pub fn a_sync<T>(a: T) -> Self
    where
        T: ImplAsync + 'static,
    {
        Action::Async(Arc::new(a))
    }

    pub fn remote<T>(a: T) -> Self
    where
        T: ImplRemote + 'static,
    {
        Action::Remote(Box::new(a))
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
/// use forester_rs::runtime::action::{recover, Impl, Tick};
/// use forester_rs::runtime::args::{RtArgs, RtValue};
/// use forester_rs::runtime::blackboard::{BBKey, BlackBoard};
/// use forester_rs::runtime::context::{TreeContext, TreeContextRef};
/// use forester_rs::runtime::{RuntimeError, TickResult};
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
///impl<T> Impl for GenerateData<T>
/// where
///      T: Fn(RtValue) -> RtValue + Send + Sync,
/// {
///     fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
///         let key = args
///             .find_or_ith("key".to_string(), 0)
///             .ok_or(RuntimeError::fail(format!(
///                 "the key is expected and should be a string"
///             )))?;
///
///         let key = key
///             .cast(ctx.clone())
///             .str()?
///             .ok_or(RuntimeError::fail(format!(
///                 "the key is expected and should be a string"
///             )))?;
///
///         let default = args
///             .find_or_ith("default".to_string(), 1)
///             .ok_or(RuntimeError::fail(format!("the default is expected")))?;
///
///         let arc_bb = ctx.bb();
///         let mut bb = arc_bb.lock()?;
///         let curr = bb.get(key.clone())?.unwrap_or(&default).clone();
///         bb.put(key, (self.generator)(curr))?;
///         Ok(TickResult::Success)
///     }
/// }
/// ```
pub trait Impl: Sync + Send {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick;
}

pub trait ImplAsync: Sync + Send {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick;
}

/// The trait to implement remote action.
/// The remote action is the action that is executed on the remote machine.
/// # Params
/// - args: the list of arguments that will be accepted from the given operation from the forester
/// - ctx: the context to send the request to the remote machine
///
/// ## Returns tick = RtResult(TickResult)
///
/// As an example, please see the `RemoteHttpAction`
///
pub trait ImplRemote: Sync + Send {
    fn tick(&self, args: RtArgs, ctx: TreeRemoteContextRef) -> Tick;
}

impl From<Box<dyn Impl>> for Action {
    fn from(value: Box<dyn Impl>) -> Self {
        Action::Sync(value)
    }
}

impl From<Arc<dyn ImplAsync>> for Action {
    fn from(value: Arc<dyn ImplAsync>) -> Self {
        Action::Async(value)
    }
}
