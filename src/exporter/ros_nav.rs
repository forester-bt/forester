use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::io::{Cursor, LineWriter};
use std::path::{Path, PathBuf};
use graphviz_rust::attributes::arrowhead::vee;
use quick_xml::events::{BytesEnd, BytesStart, Event};
use quick_xml::Writer;
use State::Started;
use crate::exporter::Exporter;
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::{RtOk, RtResult, RuntimeError};
use crate::runtime::args::RtArgument;
use crate::runtime::builder::ros_nav::find_ros_action;
use crate::runtime::rtree::rnode::{FlowType, RNode, RNodeId};


pub struct RosNavBTree {
    path: PathBuf,
}

impl RosNavBTree {
    pub fn new(path: PathBuf) -> Self {
        Self {
            path
        }
    }

    pub fn writer(&self) -> RtResult<Writer<LineWriter<File>>> {
        Ok(Writer::new_with_indent(LineWriter::new(File::create(&self.path)?), b' ', 2))
    }
}

impl Exporter for RosNavBTree {
    type Writer = Writer<LineWriter<File>>;

    fn write_terminal(&self, w: &mut Self::Writer, id: RNodeId, node: &RNode) -> RtOk {
        let action = node.name()
            .and_then(|n| n.name().ok())
            .and_then(|name| find_ros_action(name))
            .ok_or(RuntimeError::WrongArgument(format!(r#"ros analogue not found for node {:?}. Check the import "ros::nav2""#, node)))?;

        let n = action.name.as_str();
        let mut e = BytesStart::new(n);


        for RtArgument { name, value } in node.args().0 {
            e.push_attribute((name.as_str(), format!("{}", value.to_string()).as_str()));
        }

         // e.extend_attributes(attrs);

        w.write_event(Event::Empty(e))?;
        Ok(())
    }

    fn write_interior_start(&self, w: &mut Self::Writer, id: RNodeId, node: &RNode) -> RtOk {
        match node {
            RNode::Flow(FlowType::Root, n, _, _) => {
                let mut root = BytesStart::new("root");
                let name = n.name()?.as_str();
                root.push_attribute(("main_tree_to_execute", name));
                w.write_event(Event::Start(root))?;
                let mut start = BytesStart::new("BehaviorTree");
                start.push_attribute(("ID", name));
                w.write_event(Event::Start(start))?;
            }
            RNode::Decorator(_, _, _) => {}
            _ => {}
        }
        Ok(())
    }

    fn write_interior_end(&self, w: &mut Self::Writer, id: RNodeId, node: &RNode) -> RtOk {
        match node {
            RNode::Flow(FlowType::Root, n, _, _) => {
                w.write_event(Event::End(BytesEnd::new("BehaviorTree")))?;
                w.write_event(Event::End(BytesEnd::new("root")))?;
            }
            RNode::Decorator(_, _, _) => {}
            _ => {}
        }
        Ok(())
    }
}


enum State {
    Ready,
    Started,
}

