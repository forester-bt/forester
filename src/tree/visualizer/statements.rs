use graphviz_rust::attributes::{color_name, NodeAttributes, shape};
use graphviz_rust::dot_structures::*;
use graphviz_rust::dot_generator::*;
use itertools::Itertools;
use crate::tree::parser::ast::{Argument, Arguments, Key, ShortDisplayArguments, Tree, TreeType};
use crate::tree::project::invocation::Invocation;

pub trait ToStmt {
    fn to_stmt(&self, id: String) -> Stmt;
}

impl ToStmt for Tree {
    fn to_stmt(&self, id: String) -> Stmt {
        let tree_type = format!("{}", &self.tpe);
        let name = (&self.name).to_string();
        let label = NodeAttributes::label(format!("\"{} {}\"", tree_type, name));
        stmt!(node!(id; label))
    }
}

impl ToStmt for TreeType {
    fn to_stmt(&self, id: String) -> Stmt {
        let label = NodeAttributes::label(format!("\"{}\"", &self));
        let shape = shape(self);
        let color = color(self);
        stmt!(node!(id.as_str(); label,shape,color))
    }
}

fn shape(tpe: &TreeType) -> Attribute {
    match tpe {
        TreeType::Root => NodeAttributes::shape(shape::rect),
        TreeType::Parallel => NodeAttributes::shape(shape::rect),
        TreeType::Sequence => NodeAttributes::shape(shape::rect),
        TreeType::MSequence => NodeAttributes::shape(shape::rect),
        TreeType::RSequence => NodeAttributes::shape(shape::rect),
        TreeType::Fallback => NodeAttributes::shape(shape::rect),
        TreeType::RFallback => NodeAttributes::shape(shape::rect),

        TreeType::Inverter => NodeAttributes::shape(shape::tab),
        TreeType::ForceSuccess => NodeAttributes::shape(shape::tab),
        TreeType::ForceFail => NodeAttributes::shape(shape::tab),
        TreeType::Repeat => NodeAttributes::shape(shape::tab),
        TreeType::Retry => NodeAttributes::shape(shape::tab),
        TreeType::Timeout => NodeAttributes::shape(shape::tab),

        TreeType::Impl => NodeAttributes::shape(shape::component),
        TreeType::Cond => NodeAttributes::shape(shape::ellipse)
    }
}
fn color(tpe: &TreeType) -> Attribute {
    match tpe {
        TreeType::Root => NodeAttributes::color(color_name::black),
        TreeType::Parallel => NodeAttributes::color(color_name::darkred),
        TreeType::Sequence => NodeAttributes::color(color_name::darkred),
        TreeType::MSequence => NodeAttributes::color(color_name::darkred),
        TreeType::RSequence => NodeAttributes::color(color_name::darkred),
        TreeType::Fallback => NodeAttributes::color(color_name::blue),
        TreeType::RFallback => NodeAttributes::color(color_name::blue),

        TreeType::Inverter => NodeAttributes::color(color_name::purple),
        TreeType::ForceSuccess => NodeAttributes::color(color_name::purple),
        TreeType::ForceFail => NodeAttributes::color(color_name::purple),
        TreeType::Repeat => NodeAttributes::color(color_name::purple),
        TreeType::Retry => NodeAttributes::color(color_name::purple),
        TreeType::Timeout => NodeAttributes::color(color_name::purple),

        TreeType::Impl => NodeAttributes::color(color_name::green),
        TreeType::Cond => NodeAttributes::color(color_name::greenyellow),
    }
}

impl ToStmt for (TreeType, Arguments) {
    fn to_stmt(&self, id: String) -> Stmt {
        let tpe = format!("{}", &self.0);
        let args = format!("{}",ShortDisplayArguments(self.1.clone()));
        let args = if !args.is_empty() { format!("({})", args) } else { "".to_string() };
        let label = format!("\"{} {}\"", tpe, args);
        let label = NodeAttributes::label(label);
        let shape = shape(&self.0);
        let color = color(&self.0);
        stmt!(node!(id.as_str(); label,shape,color))
    }
}


impl<'a> ToStmt for Invocation<'a> {
    fn to_stmt(&self, id: String) -> Stmt {
        let tpe = format!("{}", &self.tree.tpe);
        let name = (&self.tree.name).to_string();
        let alias = self.alias.clone().unwrap_or("".to_string());
        let args = format!("{}",ShortDisplayArguments(self.arguments.clone()));
        let args = if !args.is_empty() { format!("({})", args) } else { "".to_string() };

        let full_name = if alias.is_empty() {name} else {format!("{}[{}]",alias,name)};

        let label = format!("\"{}\n{} {}\"", tpe, full_name, args);

        let label = NodeAttributes::label(label);
        let shape = shape(&self.tree.tpe);
        let color = color(&self.tree.tpe);
        stmt!(node!(id.as_str(); label,shape,color))
    }
}

