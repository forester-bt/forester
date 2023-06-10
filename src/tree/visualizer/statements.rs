use graphviz_rust::attributes::{NodeAttributes, shape};
use graphviz_rust::dot_structures::*;
use graphviz_rust::dot_generator::*;
use itertools::Itertools;
use crate::tree::parser::ast::{Argument, Arguments, Key, Tree, TreeType};
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
        stmt!(node!(id.as_str(); label,shape))
    }
}

fn shape(tpe: &TreeType) -> Attribute {
    match tpe {
        TreeType::Root => NodeAttributes::shape(shape::house),
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

impl ToStmt for (&TreeType, &Arguments) {
    fn to_stmt(&self, id: String) -> Stmt {
        let tpe = format!("{}", &self.0);
        let args = &self.1.args.iter().map(|a| {
            match a {
                Argument::Id(v) => v.clone(),
                Argument::Mes(m) => format!("{:?}", m),
                Argument::AssignedId(l, r) => format!("{}={}", l, r),
                Argument::AssignedMes(v, m) => format!("{}={:?}", v, m)
            }
        }).join(",");
        let args = if !args.is_empty() { format!("({})", args) } else { "".to_string() };
        let label = format!("\"{} {}\"", tpe, args);
        let label = NodeAttributes::label(label);
        let shape = shape(self.0);
        stmt!(node!(id.as_str(); label,shape))
    }
}


impl<'a> ToStmt for Invocation<'a> {
    fn to_stmt(&self, id: String) -> Stmt {
        let tpe = format!("{}", &self.tree.tpe);
        let name = (&self.tree.name).to_string();
        let alias = self.alias.clone().unwrap_or("".to_string());
        let args = &self.arguments.args.iter().map(|a| {
            match a {
                Argument::Id(v) => v.clone(),
                Argument::Mes(m) => format!("{:?}", m),
                Argument::AssignedId(l, r) => format!("{}={}", l, r),
                Argument::AssignedMes(v, m) => format!("{}={:?}", v, m)
            }
        }).join(",");

        let args = if !args.is_empty() { format!("({})", args) } else { "".to_string() };

        let full_name = if alias.is_empty() {name} else {format!("{}[{}]",alias,name)};

        let label = format!("\"{}\n{} {}\"", tpe, full_name, args);

        let label = NodeAttributes::label(label);
        let shape = shape(&self.tree.tpe);
        stmt!(node!(id.as_str(); label, shape))
    }
}

