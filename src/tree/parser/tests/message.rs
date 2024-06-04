use crate::tree::parser::ast::message::{Message, Number, StringLit};

use crate::tree::parser::Parser;
use parsit::test::parser_test::{expect};
use std::collections::HashMap;

#[test]
fn simple_mess() {
    let parser = Parser::new(r#"false"#).unwrap();
    expect(parser.message(0), Message::bool(false));

    let parser = Parser::new(r#"true"#).unwrap();
    expect(parser.message(0), Message::bool(true));

    let parser = Parser::new(r#"1.1"#).unwrap();
    expect(parser.message(0), Message::float(1.1));

    let parser = Parser::new(r#""v""#).unwrap();
    expect(parser.message(0), Message::str("v"));

    let parser = Parser::new(r#"[]"#).unwrap();
    expect(parser.message(0), Message::array(vec![]));

    let parser = Parser::new(r#"[{}]"#).unwrap();
    expect(
        parser.message(0),
        Message::array(vec![Message::object(vec![])]),
    );
}

#[test]
fn object() {
    let parser = Parser::new(r#"{}"#).unwrap();
    expect(parser.object(0), HashMap::new());

    let parser = Parser::new(
        r#"
    {
        "field" : 1

    }"#,
    )
    .unwrap();
    expect(
        parser.object(0),
        HashMap::from_iter(vec![("field".to_string(), Message::Num(Number::Int(1)))]),
    );

    let parser = Parser::new(
        r#"
    {
        "field" : 1,
        "field2": "v",

    }"#,
    )
    .unwrap();
    expect(
        parser.object(0),
        HashMap::from_iter(vec![
            ("field".to_string(), Message::Num(Number::Int(1))),
            (
                "field2".to_string(),
                Message::String(StringLit("v".to_string())),
            ),
        ]),
    );
}
