use std::collections::HashMap;
use parsit::test::parser_test::{expect, expect_or_env};
use crate::gol::ast::*;
use crate::gol::parser::Parser;

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
    expect(parser.message(0), Message::array(vec![Message::object(vec![])]));
}

#[test]
fn object() {
    let parser = Parser::new(r#"{}"#).unwrap();
    expect(parser.object(0), HashMap::new());

    let parser = Parser::new(r#"
    {
        "field" : 1

    }"#).unwrap();
    expect(parser.object(0), HashMap::from_iter(
        vec![
            ("field".to_string(), Message::Num(Number::Int(1)))
        ]
    ));

    let parser = Parser::new(r#"
    {
        "field" : 1,
        "field2": "v",

    }"#).unwrap();
    expect(parser.object(0), HashMap::from_iter(
        vec![
            ("field".to_string(), Message::Num(Number::Int(1))),
            ("field2".to_string(), Message::String(StringLit("v".to_string()))),
        ]
    ));
}

#[test]
fn object_with_call() {
    let parser =
        Parser::new(r#"
            {
                "field" : x(id=x),
            }"#).unwrap();


    expect(parser.message(0),Message::object(vec![
        ("field".to_string(),
         Message::Call(
             Call::invocation(
                 "x",
                 Arguments::new(vec![Argument::id_id("id","x")])
             )
         )
        )
    ]));

    let parser =
        Parser::new(r#"
            {
                "field" : fallback { call1() sequence call3() },
            }"#).unwrap();


    expect(parser.message(0),Message::object(vec![
        ("field".to_string(),
         Message::Call(
             Call::lambda(TreeType::Fallback,Calls::new(vec![
                 Call::invocation("call1",Arguments::default()),
                 Call::Lambda(TreeType::Sequence,
                              Calls::new(vec![Call::invocation("call3",Arguments::default())]))
             ]))
         )
        )
    ]));
}

