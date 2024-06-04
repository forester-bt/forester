


use crate::runtime::action::{Action, ActionName};
use crate::runtime::{RtResult, RuntimeError};
use crate::runtime::ros::OneTimeSender;


pub(crate) fn action_impl(action: &ActionName) -> RtResult<Action> {
    match action.as_str() {
        "publish" => Ok(Action::sync(OneTimeSender::Publish)),
        "advertise" => Ok(Action::sync(OneTimeSender::Advertise)),
        "unsubscribe" => Ok(Action::sync(OneTimeSender::Unsubscribe)),
        "subscribe" => Ok(Action::sync(OneTimeSender::Subscribe)),
        _ => Err(RuntimeError::UnImplementedAction(format!("the action is not found {}", action)))
    }
}

pub fn ros_actions_file() -> String {
    format!(r#"// Ros2 Core specific actions and decorators.
// The actions are accessible using the import 'import "ros::core"'

// Publish message to the topic
impl publish(topic:string, value:any, url:string);
// Advertise topic
impl advertise(topic:string, tp:string, url:string);
// Unsubscribe topic
impl unsubscribe(topic:string, url:string);

// Subscribe to the topic
// source_cfg:tp:string,
//     throttle_rate:num,
//     queue_length:num,
//     fragment_size:num,
//     compression:string,
//
// target_cfg: daemon type for subscriber - the way how the daemon will update bb
//   - tp: type of the daemon : 'last', buffer, 'all'
//   - buf_size: size of the buffer if the type is buffer
//   - dst: destination bb

impl subscribe(
    topic:string,
    source_cfg:object,
    target_cfg:object,
    url:string
);

"#)
}
