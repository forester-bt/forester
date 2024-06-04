//! Provide a DDS implementation for the runtime.
// ros2 launch rosbridge_server rosbridge_websocket_launch.xml
pub mod client;








use crate::runtime::action::{Impl, Tick};
use crate::runtime::args::{RtArgs, RtValue};
use crate::runtime::context::TreeContextRef;
use crate::runtime::{RtResult, RuntimeError, TickResult};
use crate::runtime::blackboard::BBKey;
use crate::runtime::env::daemon::context::DaemonContext;
use crate::runtime::env::daemon::{Daemon, DaemonFn, StopFlag};
use crate::runtime::ros::client::{SubscribeCfg, WS};

/// abilities
/// - subscribe to topics - > get data and put it to bb.
/// - publish to topics
/// - pass some params to services
/// - call services and nodes
pub enum OneTimeSender {
    Publish,
    Advertise,
    Unsubscribe,
    Subscribe,
}

impl Impl for OneTimeSender {
    fn tick(&self, args: RtArgs, ctx: TreeContextRef) -> Tick {
        match self {
            OneTimeSender::Publish => {
                let topic = param_as_str("topic", 0, &args, ctx.clone())?;
                let mes = args.find_or_ith("value".to_string(), 1)
                    .ok_or(RuntimeError::fail(format!("the value is not found")))?;
                let url = param_as_str("url", 2, &args, ctx)?;
                client::publish(topic, mes, url)
            }
            OneTimeSender::Advertise => {
                let topic = param_as_str("topic", 0, &args, ctx.clone())?;
                let tp = param_as_str("type", 1, &args, ctx.clone())?;
                let url = param_as_str("url", 2, &args, ctx)?;

                client::advertise(topic, tp, url)
            }
            OneTimeSender::Unsubscribe => {
                let topic = param_as_str("topic", 0, &args, ctx.clone())?;
                let url = param_as_str("url", 2, &args, ctx.clone())?;
                client::unsubscribe(topic.clone(), url.clone())?;

                let rt_env_ref = ctx.env();
                let mut rt_env = rt_env_ref.lock()?;
                rt_env.stop_daemon(&format!("{url}_{topic}"));

                Ok(TickResult::success())
            }
            OneTimeSender::Subscribe => {
                let topic = param_as_str("topic", 0, &args, ctx.clone())?;
                let s_cfg = param_as("source_cfg", 1, &args, SubscribeCfg::from)?;
                let t_cfg = param_as("target_cfg", 2, &args, TargetCfg::from)?;
                let url = param_as_str("url", 3, &args, ctx.clone())?;

                let ws = client::subscribe(topic.clone(), s_cfg, url.clone())?;

                ctx.env().lock()?.start_named_daemon(
                    format!("{url}_{topic}"),
                    Daemon::sync(SubscriberDaemon::new(t_cfg, ws)),
                    DaemonContext::from(ctx.clone()),
                )?;

                Ok(TickResult::success())
            }
        }
    }
}

pub enum SubscriberDaemon {
    Last(WS, BBKey),
    All(WS, BBKey),
}


impl SubscriberDaemon {
    pub fn new(cfg: TargetCfg, ws: WS) -> Self {
        match cfg.tp.as_str() {
            "last" => SubscriberDaemon::Last(ws, cfg.dst),
            _ => SubscriberDaemon::All(ws, cfg.dst),
        }
    }
    pub fn ws(&self) -> &WS {
        match self {
            SubscriberDaemon::Last(ws, _) => ws,
            SubscriberDaemon::All(ws, _) => ws,
        }
    }
}

impl DaemonFn for SubscriberDaemon {
    fn perform(&mut self, _ctx: DaemonContext, _signal: StopFlag) {
        // loop {
        //     if signal.load(Ordering::Relaxed) {
        //         break;
        //     }
        //     let msg = self.ws().read().unwrap();
        //
        //     match self {
        //         SubscriberDaemon::Last(_, key) => {
        //             if msg.is_text() {
        //                 let string = msg.into_text().expect("text");
        //                 let value = RtValue::deserialize(string).unwrap();
        //                 ctx.bb.lock().unwrap().put(key.clone(), value).unwrap();
        //             } else {
        //                 debug!(target: "ws-subscriber" ,"Subscriber Daemon: Received not text: {:#?}", msg);
        //             }
        //         }
        //         SubscriberDaemon::All(_, key) => {
        //             if msg.is_text() {
        //                 let string = msg.into_text().expect("text");
        //                 let value = RtValue::deserialize(string).unwrap();
        //                 push_to_arr(ctx.bb.clone(), key.clone(), value).unwrap();
        //             } else {
        //                 debug!(target: "ws-subscriber" ,"Subscriber Daemon: Received not text: {:#?}", msg);
        //             }
        //         }
        //     }
        //
        //
        //     // let msg = serde_json::from_str::<ros::RosMessage>(&msg)?;
        //     // let mut bb = ctx.bb.lock()?;
        //     // bb.insert(topic.clone(), RtValue::from(msg));
        // }
    }
}

#[derive(Debug, Default)]
pub struct TargetCfg {
    tp: String,
    buf_size: Option<usize>,
    dst: String,
}

impl TargetCfg {
    pub fn from(v: RtValue) -> RtResult<TargetCfg> {
        let elems = v.as_map(|(k, v)| (k, v))
            .ok_or(RuntimeError::fail(format!("the target_cfg should be an object")))?;

        let tp = elems.get("tp").and_then(|v| v.clone().as_string())
            .ok_or(RuntimeError::fail(format!("the tp is not found")))?;

        let buf_size = elems.get("buf_size")
            .and_then(|v| v.clone().as_int().map(|v| v as usize));

        let dst = elems.get("dst").and_then(|v| v.clone().as_string())
            .ok_or(RuntimeError::fail(format!("the dst should be a string")))?;

        Ok(TargetCfg { tp, buf_size, dst })
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

fn param_as<T, V>(key: &str, i: usize, args: &RtArgs, m: T) -> RtResult<V>
    where T: Fn(RtValue) -> RtResult<V>
{
    args
        .find_or_ith(key.to_string(), i)
        .ok_or(RuntimeError::fail(format!("the {key} is not found")))
        .and_then(|v| m(v))
}
