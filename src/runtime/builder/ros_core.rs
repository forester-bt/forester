use std::collections::HashMap;
use std::fmt::{Display, Formatter};
use itertools::Itertools;
use crate::runtime::action::{Action, ActionName};
use crate::runtime::action::builtin::ReturnResult;
use crate::runtime::{RtResult, RuntimeError};
use crate::tree::parser::ast::arg::{MesType, Param};
use crate::tree::parser::ast::message::Message;


pub(super) fn action_impl(action: &ActionName) -> RtResult<Action> {
    Ok()
}

pub fn ros_actions_file() -> String {
    format!(r#"// Ros2 Core specific actions and decorators.
// The actions are accessible using the import 'import "ros::core"'

// Publish message to the topic
impl pub_str(v:string, topic:string);



"#)
}
