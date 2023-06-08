use std::collections::HashMap;
use parsit::error::ParseError;
use parsit::test::parser_test::*;
use crate::tree::ast::*;
use crate::tree::parser::Parser;
use crate::tree::project::File;

#[test]
fn file() {
    let parser = Parser::new(r#"
import "nested/impls.tree"

root ball fallback {
    try_to_place_to() // the objects in bb that denote ball and bin
    ask_for_help()
}

sequence try_to_place_to(obj:object){
    fallback {
        find_ball(obj)
    }

}

cond grasped(obj:object)

    "#).unwrap();
    expect(parser.file(0), AstFile(vec![
        FileEntity::Import(Import::file("nested/impls.tree")),
        FileEntity::Tree(Tree::new(TreeType::Root, Key("ball".to_string()),
                                   Params::default(), Calls::new(vec![
                Call::lambda(TreeType::Fallback,Calls::new(vec![
                    Call::invocation("try_to_place_to", Arguments::default()),
                    Call::invocation("ask_for_help", Arguments::default()),
                ]))
            ]))),
        FileEntity::Tree(Tree::new(TreeType::Sequence, Key("try_to_place_to".to_string()),
                                   Params::new(vec![Param::new("obj", MesType::Object)]), Calls::new(vec![
                Call::lambda(TreeType::Fallback, Calls::new(vec![
                    Call::invocation("find_ball", Arguments::new(vec![Argument::id("obj")]))
                ]))
            ]))),
        FileEntity::Tree(Tree::new(TreeType::Cond, Key("grasped".to_string()),
                                   Params::new(vec![Param::new("obj", MesType::Object)]),
                                   Calls::default())),
    ]));
}

#[test]
fn short_description_file() {
    let parser = Parser::new(r#"
cond grasped {}
cond ball_found {}

    "#).unwrap();
    expect(parser.file(0), AstFile(vec![
        FileEntity::Tree(Tree::new(
            TreeType::Cond,
            Key("grasped".to_string()), Params::default(),
            Calls::default()
        )),
        FileEntity::Tree(Tree::new(
            TreeType::Cond,
            Key("ball_found".to_string()), Params::default(),
            Calls::default()
        ))
    ]));
}


