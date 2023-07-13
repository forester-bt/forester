use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::TreeContext;
use crate::runtime::{RuntimeError, TickResult};
use reqwest::Error;

pub struct HttpGet;

impl Impl for HttpGet {
    fn tick(&self, args: RtArgs, ctx: &mut TreeContext) -> Tick {
        let url = args
            .find_or_ith("url".to_string(), 0)
            .and_then(RtValue::as_string)
            .ok_or(RuntimeError::fail(
                "url is not found or it is not a string".to_string(),
            ))?;
        let out = args
            .find_or_ith("bb_key".to_string(), 1)
            .and_then(RtValue::as_string)
            .ok_or(RuntimeError::fail(
                "bb_key is not found or it is not a string".to_string(),
            ))?;

        match reqwest::blocking::get(url).and_then(|v| v.text()) {
            Ok(resp) => {
                ctx.bb().put(out, RtValue::str(resp))?;
                Ok(TickResult::success())
            }
            Err(err) => Ok(TickResult::failure(format!("error {}", err))),
        }
    }
}
