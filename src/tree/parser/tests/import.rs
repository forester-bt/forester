use crate::tree::parser::ast::*;
use crate::tree::parser::Parser;

use parsit::test::parser_test::*;


#[test]
fn import() {
    let parser = Parser::new(
        r#"
    import "nested.tree/impls.tree"
    "#,
    )
    .unwrap();
    expect(parser.import(0), Import::file("nested.tree/impls.tree"));
}

#[test]
fn import_names() {
    let parser = Parser::new(
        r#"
    import "nested.tree/impls.tree" {
        first,
        second,
    }
    "#,
    )
    .unwrap();
    expect(
        parser.import(0),
        Import::names("nested.tree/impls.tree", vec!["first", "second"]),
    );
}
#[test]
fn import_names_alias() {
    let parser = Parser::new(
        r#"
    import "nested.tree/impls.tree" {
        first => f,
        second,
        third => t,
    }
    "#,
    )
    .unwrap();
    expect(
        parser.import(0),
        Import::names_mixed(
            "nested.tree/impls.tree",
            vec![
                ImportName::alias("first", "f"),
                ImportName::id("second"),
                ImportName::alias("third", "t"),
            ],
        ),
    );
}
