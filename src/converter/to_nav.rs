
use std::fs::File;
use std::io::{LineWriter};
use std::path::{PathBuf};
use quick_xml::events::{BytesEnd, BytesStart, Event};
use quick_xml::Writer;
use crate::converter::{Converter};
use crate::runtime::rtree::RuntimeTree;
use crate::runtime::{RtOk, RtResult, RuntimeError};
use crate::runtime::args::{RtArgument, RtValue};
use crate::runtime::builder::ros_nav::{find_ros_action};
use crate::runtime::rtree::rnode::{DecoratorType, FlowType, RNode, RNodeId};

enum State {
    Ready,
    Started,
}

pub struct ToRosNavConverter<'a> {
    tree: &'a RuntimeTree,
    xml: PathBuf
}

impl<'a> Converter for ToRosNavConverter<'a> {
    type Output = ();

    fn convert(&self) -> RtOk {
        let mut stack = vec![(self.tree.root, State::Ready)];
        let mut writer = self.writer()?;
        while let Some((id, st)) = stack.pop() {
            let node = self.tree.node(&id)?;
            if node.is_action() {
                &self.write_terminal(&mut writer, id, node)?;
            } else {
                match st {
                    State::Ready => {
                        &self.write_interior_start(&mut writer, id, node)?;
                        stack.push((id, State::Started));
                        for child in node.children().iter().rev() {
                            stack.push((*child, State::Ready));
                        }
                    }
                    State::Started => {
                        &self.write_interior_end(&mut writer, id, node)?;
                    }
                }
            }
        }


        Ok(())
    }
}

impl<'a> ToRosNavConverter<'a> {
    pub fn new(tree: &'a RuntimeTree, xml: PathBuf) -> Self {
        Self { tree, xml }
    }
    /// Write the terminal node into the specific format. The terminal node is the node that has no children by contract (actions)
    fn write_terminal(&self, w: &mut  Writer<LineWriter<File>>, _id: RNodeId, node: &RNode) -> RtOk {
        let action = node.name()
            .and_then(|n| n.name().ok())
            .and_then(|name| find_ros_action(name))
            .ok_or(RuntimeError::WrongArgument(format!(r#"ros analogue not found for node {:?}. Check the import "ros::nav2""#, node)))?;

        let n = action.name.as_str();
        let mut e = BytesStart::new(n);
        handle_attrs(node.args().0, &mut e)?;
        w.write_event(Event::Empty(e))?;
        Ok(())
    }
    // Write the opening for an interior node into the specific format.
    /// The interior node is the node that has children by contract (flows)
    fn write_interior_start(&self, w: &mut  Writer<LineWriter<File>>, _id: RNodeId, node: &RNode) -> RtOk {
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
            RNode::Flow(FlowType::Sequence, name, _, _) => {
                if name.has_name() {
                    let name = name.name()?;
                    match find_ros_action(name) {
                        None => {
                            let mut e = BytesStart::new("PipelineSequence");
                            e.push_attribute(("name", name.as_str()));
                            w.write_event(Event::Start(e))?;
                        }
                        Some(action) => {
                            let mut e = BytesStart::new(action.name.as_str());
                            handle_attrs(node.args().0, &mut e)?;
                            w.write_event(Event::Start(e))?;
                        }
                    }
                } else {
                    w.write_event(Event::Start(BytesStart::new("PipelineSequence")))?;
                }
            }
            RNode::Flow(FlowType::Fallback, name, _, _) => {
                let mut e = BytesStart::new("RoundRobin");
                if name.has_name() {
                    e.push_attribute(("name", name.name()?.as_str()));
                }
                w.write_event(Event::Start(e))?;
            }
            RNode::Flow(FlowType::RFallback, name, _, _) => {
                let mut e = BytesStart::new("ReactiveFallback");
                if name.has_name() {
                    e.push_attribute(("name", name.name()?.as_str()));
                }
                w.write_event(Event::Start(e))?;
            }
            RNode::Decorator(DecoratorType::Retry, args, _) => {
                let mut e = BytesStart::new("RecoveryNode");
                let a = args.0.first()
                    .ok_or(RuntimeError::WrongArgument(format!(r#"decorator {:?} does not have arguments"#, node)))?;
                e.push_attribute(("number_of_retries", a.clone().val().to_string().as_str()));
                w.write_event(Event::Start(e))?;
            }
            node => {
                File::create(&self.xml)?;
                return Err(RuntimeError::IOError(format!(r#"export to xml error: The node {:?} does not have analogue in nav2 "#, node)));
            }
        }
        Ok(())
    }
    /// Write the closing for an interior node into the specific format.
    fn write_interior_end(&self, w: &mut  Writer<LineWriter<File>>, _id: RNodeId, node: &RNode) -> RtOk {
        match node {
            RNode::Flow(FlowType::Root, _, _, _) => {
                w.write_event(Event::End(BytesEnd::new("BehaviorTree")))?;
                w.write_event(Event::End(BytesEnd::new("root")))?;
            }
            RNode::Flow(FlowType::Sequence, name, _, _) => {
                if name.has_name() {
                    let name = name.name()?;
                    match find_ros_action(name) {
                        None => {
                            w.write_event(Event::End(BytesEnd::new("PipelineSequence")))?;
                        }
                        Some(action) => {
                            w.write_event(Event::End(BytesEnd::new(action.name.as_str())))?;
                        }
                    }
                } else {
                    w.write_event(Event::End(BytesEnd::new("PipelineSequence")))?;
                }
            }
            RNode::Flow(FlowType::Fallback, _, _, _) => {
                w.write_event(Event::End(BytesEnd::new("RoundRobin")))?;
            }
            RNode::Flow(FlowType::RFallback, _, _, _) => {
                w.write_event(Event::End(BytesEnd::new("ReactiveFallback")))?;
            }
            RNode::Decorator(DecoratorType::Retry, _, _) => {
                w.write_event(Event::End(BytesEnd::new("RecoveryNode")))?;
            }
            _ => {}
        }
        Ok(())
    }

    fn writer(&self) -> RtResult<Writer<LineWriter<File>>> {
        Ok(Writer::new_with_indent(LineWriter::new(File::create(&self.xml)?), b' ', 2))
    }
}






fn handle_attrs(attrs: Vec<RtArgument>, e: &mut BytesStart) -> RtOk {
    for RtArgument { name, value } in attrs {
       match value {
            RtValue::Pointer(v) => {
                e.push_attribute((name.as_str(), format!(r#"{{{}}}"#, v.to_string()).as_str()));
            }
            RtValue::Call(_) => {}
            _ => {
                e.push_attribute((name.as_str(), format!("{}", value.to_string()).as_str()));
            }
        };
    }
    Ok(())
}


