use crate::runtime::action::{Impl, ImplAsync, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::TreeContext;
use crate::runtime::context::TreeContextRef;
use crate::runtime::{RtResult, RuntimeError, TickResult};
use reqwest::Error;
use std::future::Future;
use tokio::runtime::Runtime;
use tokio::task::{JoinError, JoinHandle};
/// Synchronious http get.
/// It accepts url for request and key in bb to write the results as string.
pub struct HttpGet;

impl HttpGet {
    fn on_tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        let url = args
            .find_or_ith("url".to_string(), 0)
            .ok_or(RuntimeError::fail(
                "url is not found or it is not a string".to_string(),
            ))
            .and_then(|v| v.cast(ctx.clone()).str())?
            .ok_or(RuntimeError::fail(
                "url is not found or it is not a string".to_string(),
            ))?;

        let out = args
            .find_or_ith("bb_key".to_string(), 1)
            .ok_or(RuntimeError::fail(
                "bb_key is not found or it is not a string".to_string(),
            ))
            .and_then(|v| v.cast(ctx.clone()).str())?
            .ok_or(RuntimeError::fail(
                "bb_key is not found or it is not a string".to_string(),
            ))?;

        match reqwest::blocking::get(url).and_then(|v| v.text()) {
            Ok(resp) => {
                ctx.bb().lock()?.put(out, RtValue::str(resp))?;
                Ok(TickResult::success())
            }
            Err(err) => Ok(TickResult::failure(format!("error {}", err))),
        }
    }
}

impl Impl for HttpGet {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        self.on_tick(args, ctx)
    }
}

impl ImplAsync for HttpGet {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        self.on_tick(args, ctx)
    }
}
