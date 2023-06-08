use std::collections::HashMap;
use parsit::error::ParseError;
use parsit::test::parser_test::*;
use crate::tree::ast::*;
use crate::tree::parser::Parser;

#[test]
fn import() {
    let parser = Parser::new(r#"
    import "nested/impls.tree"
    "#).unwrap();
    expect(parser.import(0), Import::file("nested/impls.tree"));
}

#[test]
fn import_names() {
    let parser = Parser::new(r#"
    import "nested/impls.tree" {
        first,
        second,
    }
    "#).unwrap();
    expect(parser.import(0), Import::names("nested/impls.tree",vec!["first","second"]));
}
#[test]
fn import_names_alias() {
    let parser = Parser::new(r#"
    import "nested/impls.tree" {
        first => f,
        second,
        third => t,
    }
    "#).unwrap();
    expect(parser.import(0), Import::names_mixed("nested/impls.tree",vec![
        ImportName::alias("first","f"),
        ImportName::id("second"),
        ImportName::alias("third","t"),
    ]));
}


