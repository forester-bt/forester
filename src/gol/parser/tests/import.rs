use std::collections::HashMap;
use parsit::error::ParseError;
use parsit::test::parser_test::*;
use crate::gol::ast::*;
use crate::gol::parser::Parser;

#[test]
fn import() {
    let parser = Parser::new(r#"
    import "nested/impls.gol"
    "#).unwrap();
    expect(parser.import(0), Import::file("nested/impls.gol"));
}

#[test]
fn import_names() {
    let parser = Parser::new(r#"
    import "nested/impls.gol" {
        first,
        second,
    }
    "#).unwrap();
    expect(parser.import(0), Import::names("nested/impls.gol",vec!["first","second"]));
}
#[test]
fn import_names_alias() {
    let parser = Parser::new(r#"
    import "nested/impls.gol" {
        first => f,
        second,
        third => t,
    }
    "#).unwrap();
    expect(parser.import(0), Import::names_mixed("nested/impls.gol",vec![
        ImportName::alias("first","f"),
        ImportName::id("second"),
        ImportName::alias("third","t"),
    ]));
}


