use graphviz_rust::attributes::NodeAttributes;
use graphviz_rust::dot_structures::*;
use graphviz_rust::dot_generator::*;
use itertools::Itertools;
use crate::gol::ast::{Argument, Arguments, Key, Tree, TreeType};

pub trait ToStmt {
    fn to_stmt(&self, id: String) -> Stmt;
}

impl ToStmt for Tree {
    fn to_stmt(&self, id: String) -> Stmt {
        let tree_type = format!("{}", &self.tpe);
        let name = (&self.name.0).to_string();


        let label = NodeAttributes::label(format!("\"{} {}\"", tree_type, name));


        stmt!(node!(id; label))
    }
}

impl ToStmt for TreeType {
    fn to_stmt(&self, id: String) -> Stmt {
        let label = NodeAttributes::label(format!("{}", &self));
        stmt!(node!(id.as_str(); label))
    }
}

impl ToStmt for (&TreeType, &Arguments) {
    fn to_stmt(&self, id: String) -> Stmt {
        let tpe = format!("{}", &self.0);
        let args = &self.1.args.iter().map(|a| {
            match a {
                Argument::Id(Key(v)) => v.clone(),
                Argument::Mes(m) => format!("{:?}", m),
                Argument::AssignedId(Key(l), Key(r)) => format!("{}={}", l, r),
                Argument::AssignedMes(Key(v), m) => format!("{}={:?}", v, m)
            }
        }).join(",");
        let label = format!("{} ({})",tpe,args);
        let label = NodeAttributes::label(label);

        stmt!(node!(id.as_str(); label))
    }
}
