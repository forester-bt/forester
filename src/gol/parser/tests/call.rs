use std::collections::HashMap;
use parsit::error::ParseError;
use parsit::test::parser_test::*;
use crate::gol::ast::*;
use crate::gol::parser::Parser;

#[test]
fn invocation() {
    let parser = Parser::new(r#"call()"#).unwrap();
    expect(parser.call(0), Call::invocation("call", Arguments::default()));

    let parser = Parser::new(r#"call(id)"#).unwrap();
    expect(parser.call(0),
           Call::invocation("call",
                            Arguments {
                                args: vec![
                                    Argument::id("id")
                                ]
                            },
           ));
    let parser = Parser::new(r#"call(id=id1)"#).unwrap();
    expect(parser.call(0),
           Call::invocation("call",
                            Arguments {
                                args: vec![
                                    Argument::id_id("id", "id1")
                                ]
                            },
           ));

    let parser = Parser::new(r#"call(id2=3)"#).unwrap();
    expect(parser.call(0),
           Call::invocation("call",
                            Arguments {
                                args: vec![
                                    Argument::id_mes("id2", Message::int(3))
                                ]
                            },
           ));
    let parser = Parser::new(r#"call(id2=3, id3=id4)"#).unwrap();
    expect(parser.call(0),
           Call::invocation("call",
                            Arguments {
                                args: vec![
                                    Argument::id_mes("id2", Message::int(3)),
                                    Argument::id_id("id3", "id4"),
                                ]
                            },
           ));
}

#[test]
fn lambda() {
    let parser = Parser::new(r#"call()"#).unwrap();
    expect(parser.call(0), Call::invocation("call", Arguments::default()));

    let parser = Parser::new(r#"call()"#).unwrap();
    expect(parser.call(0), Call::invocation("call", Arguments::default()));

    let parser = Parser::new(r#"ttype name {} "#).unwrap();
    fail(parser.call(0));

    let parser = Parser::new(r#"root {} "#).unwrap();
    expect(parser.call(0), Call::lambda(TreeType::Root, Calls::default()));

    let parser = Parser::new(r#"root { name() } "#).unwrap();
    expect(parser.call(0), Call::lambda(TreeType::Root, Calls::new(vec![
        Call::invocation("name", Arguments::default())
    ])));
    let parser = Parser::new(r#"root name() "#).unwrap();
    expect(parser.call(0), Call::lambda(TreeType::Root, Calls::new(vec![
        Call::invocation("name", Arguments::default())
    ])));

    let parser = Parser::new(r#"impl { call()} "#).unwrap();
    assert_eq!(parser.call(0).error(), Some(ParseError::FailedOnValidation(
        "the types impl or cond should have declaration and get called by name",
        6)));
}

#[test]
fn decorator() {
    let parser = Parser::new(r#"inverter() call()"#).unwrap();
    expect(parser.call(0),
           Call::decorator(
               TreeType::Inverter,
               Arguments::default(),
               Call::invocation("call", Arguments::default()),
           ),
    );
    let parser = Parser::new(r#"inverter() {call()}"#).unwrap();
    expect(parser.call(0),
           Call::decorator(
               TreeType::Inverter,
               Arguments::default(),
               Call::invocation("call", Arguments::default()),
           ),
    );

    let parser = Parser::new(r#"inverter fallback { call1() sequence { call1() call2()} }"#).unwrap();
    expect(parser.call(0),
           Call::decorator(
               TreeType::Inverter,
               Arguments::default(),
               Call::lambda(TreeType::Fallback, Calls::new(vec![
                   Call::invocation("call1", Arguments::default()),
                   Call::lambda(TreeType::Sequence, Calls::new(vec![
                       Call::invocation("call1", Arguments::default()),
                       Call::invocation("call2", Arguments::default()),
                   ])),
               ])),
           ),
    );

    let parser = Parser::new(r#"timeout(5) call()"#).unwrap();
    expect(parser.call(0),
           Call::decorator(
               TreeType::Timeout,
               Arguments::new(vec![Argument::mes(Message::Num(Number::Int(5)))]),
               Call::invocation("call", Arguments::default()),
           ),
    );

    let parser = Parser::new(r#"inverter { call() call2 ()} "#).unwrap();
    assert_eq!(parser.call(0).error(), Some(ParseError::FailedOnValidation(
        "any decorator should have only one child",
        9)));
}

#[test]
fn calls() {
    let txt = r#"
    {
        fallback {
            ball_found(obj)
            find_ball(obj)
        }
        fallback {
            close(obj)
            approach(obj)
        }

    }
    "#;
    let parser = Parser::new(txt).unwrap();
    expect(parser.calls(0),
           Calls::new(vec![
               Call::Lambda(TreeType::Fallback, Calls::new(vec![
                   Call::invocation("ball_found", Arguments::new(vec![Argument::id("obj")])),
                   Call::invocation("find_ball", Arguments::new(vec![Argument::id("obj")])),
               ])),
               Call::Lambda(TreeType::Fallback, Calls::new(vec![
                   Call::invocation("close", Arguments::new(vec![Argument::id("obj")])),
                   Call::invocation("approach", Arguments::new(vec![Argument::id("obj")])),
               ]))
           ]),
    );
}


