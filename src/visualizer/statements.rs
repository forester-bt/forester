use crate::runtime::args::ShortDisplayedRtArguments;
use crate::runtime::rnode::{FlowType, RNode, RNodeName};
use crate::tree::parser::ast::{Argument, Arguments, Key, Tree, TreeType};
use crate::tree::project::invocation::Invocation;
use graphviz_rust::attributes::{color_name, shape, NodeAttributes};
use graphviz_rust::dot_generator::*;
use graphviz_rust::dot_structures::*;
use itertools::Itertools;
use std::fmt::format;

pub trait ToStmt {
    fn to_stmt(&self, id: String) -> Stmt;
}

fn name_to_label(name: &RNodeName) -> String {
    match name {
        RNodeName::Lambda => "".to_string(),
        RNodeName::Name(name) => format!("{name}"),
        RNodeName::Alias(n, a) => format!("{n}[{a}]"),
    }
}

impl ToStmt for RNode {
    fn to_stmt(&self, id: String) -> Stmt {
        match self {
            RNode::Leaf(name, args) => {
                let label = NodeAttributes::label(format!(
                    "\"{} {}\"",
                    name_to_label(name),
                    ShortDisplayedRtArguments(args)
                ));
                let color = NodeAttributes::color(color_name::green);
                let shape = NodeAttributes::shape(shape::component);

                stmt!(node!(id.as_str(); label, shape, color))
            }
            RNode::Flow(t, name, args, _) => {
                let color = flow_color(t);
                let shape = NodeAttributes::shape(shape::rect);

                let name_s = name_to_label(name);
                let args_s = ShortDisplayedRtArguments(args).to_string();

                let label = if name_s.is_empty() && args_s.is_empty() {
                    NodeAttributes::label(format!("\"{}\"", t,))
                } else {
                    NodeAttributes::label(format!("\"{}\n{} {}\"", t, name_s, args_s))
                };

                stmt!(node!(id.as_str(); label,shape,color))
            }
            RNode::Decorator(t, args, _) => {
                let label =
                    NodeAttributes::label(format!("\"{} {}\"", t, ShortDisplayedRtArguments(args)));
                let color = NodeAttributes::color(color_name::purple);
                let shape = NodeAttributes::shape(shape::tab);
                stmt!(node!(id.as_str(); label,shape,color))
            }
        }
    }
}

fn flow_color(tpe: &FlowType) -> Attribute {
    match tpe {
        FlowType::Root => NodeAttributes::color(color_name::black),
        FlowType::Parallel => NodeAttributes::color(color_name::darkred),
        FlowType::Sequence => NodeAttributes::color(color_name::darkred),
        FlowType::MSequence => NodeAttributes::color(color_name::darkred),
        FlowType::RSequence => NodeAttributes::color(color_name::darkred),
        FlowType::Fallback => NodeAttributes::color(color_name::blue),
        FlowType::RFallback => NodeAttributes::color(color_name::blue),
    }
}
