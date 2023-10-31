//! Provide a DDS implementation for the runtime.
// ros2 launch rosbridge_server rosbridge_websocket_launch.xml
pub mod ws_client;

use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::TreeContextRef;
use crate::runtime::TickResult;

/// abilities
/// - subscribe to topics - > get data and put it to bb.
/// - publish to topics
/// - pass some params to services
/// - call services and nodes
pub enum OneTimePublisher{
    DefaultQOS,
    CustomQOS,
}

impl Impl for OneTimePublisher {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {


        Ok(TickResult::success())
    }
}

#[derive(Clone, Debug, Default)]
pub struct ForesterRosMessage {
    tp: String,
    msg: String,
}