//! Provide a DDS implementation for the runtime.
// ros2 launch rosbridge_server rosbridge_websocket_launch.xml
pub mod client;

use std::fmt::format;
use tungstenite::{connect, Message};
use url::Url;
use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::TreeContextRef;
use crate::runtime::{ros, RtResult, RuntimeError, TickResult};

/// abilities
/// - subscribe to topics - > get data and put it to bb.
/// - publish to topics
/// - pass some params to services
/// - call services and nodes
pub enum OneTimePublisher {
    Publish,
    Advertise,
}

impl Impl for OneTimePublisher {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        match self {
            OneTimePublisher::Publish => {
                let topic = param_as_str("topic", 0, &args, ctx.clone())?;
                let mes = args.find_or_ith("value".to_string(), 1)
                    .ok_or(RuntimeError::fail(format!("the value is not found")))?;
                let url = param_as_str("url", 2, &args, ctx)?;
                client::publish(topic, mes, url)
            }
            OneTimePublisher::Advertise => {
                let topic = param_as_str("topic", 0, &args, ctx.clone())?;
                let tp = param_as_str("type", 1, &args, ctx.clone())?;
                let url = param_as_str("url", 2, &args, ctx)?;

                client::advertise(topic, tp, url)
            }
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct ForesterRosMessage {
    tp: String,
    msg: String,
}


fn param_as_str(key: &str, i: usize, args: &RtArgs, ctx: TreeContextRef) -> RtResult<String> {
    args
        .find_or_ith(key.to_string(), i)
        .ok_or(RuntimeError::fail(format!("the {key} is not found")))
        .and_then(|v| v.cast(ctx).str())
        .and_then(|v| v.ok_or(RuntimeError::fail(format!("the {key} should be a string"))))
}
