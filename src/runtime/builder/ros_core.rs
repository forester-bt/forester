use std::collections::HashMap;
use std::fmt::{Display, Formatter};
use itertools::Itertools;
use crate::runtime::action::{Action, ActionName};
use crate::runtime::{RtResult, RuntimeError};
use crate::runtime::ros::OneTimePublisher;


pub(crate) fn action_impl(action: &ActionName) -> RtResult<Action> {
    match action.as_str() {
        "publish" => Ok(Action::sync(OneTimePublisher::Publish)),
        "advertise" => Ok(Action::sync(OneTimePublisher::Advertise)),
        _ => Err(RuntimeError::UnImplementedAction(format!("the action is not found {}", action)))
    }
}

pub fn ros_actions_file() -> String {
    format!(r#"// Ros2 Core specific actions and decorators.
// The actions are accessible using the import 'import "ros::core"'

// Publish message to the topic
impl publish(topic:string, value:any, url:string);
impl advertise(topic:string, tp:string, url:string);
"#)
}
