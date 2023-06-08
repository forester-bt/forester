use parsit::test::parser_test::expect;
use crate::tree::ast::*;
use crate::tree::parser::Parser;

#[test]
fn plain_arg() {
    let parser = Parser::new(r#"ball"#).unwrap();
    expect(parser.arg(0), Argument::id("ball"));

    let parser = Parser::new(r#"ball=ball"#).unwrap();
    expect(parser.arg(0), Argument::id_id("ball", "ball"));

    let parser = Parser::new(r#"1"#).unwrap();
    expect(parser.arg(0), Argument::mes(Message::int(1)));
    let parser = Parser::new(r#""1""#).unwrap();
    expect(parser.arg(0), Argument::mes(Message::str("1")));

    let parser = Parser::new(r#"[true,false]"#).unwrap();
    expect(parser.arg(0), Argument::mes(Message::array(vec![Message::bool(true), Message::bool(false)])));

    let parser = Parser::new(r#"x = [true,false]"#).unwrap();
    expect(parser.arg(0), Argument::id_mes("x",
                                           Message::array(vec![Message::bool(true), Message::bool(false)])));

    let parser = Parser::new(r#"x()"#).unwrap();
    expect(parser.arg(0), Argument::mes(Message::invocation("x",
                                                            Arguments::default())),
    );
}

#[test]
fn call_arg() {
    let parser = Parser::new(r#"a = x()"#).unwrap();
    expect(parser.arg(0),
           Argument::id_mes(
               "a",
               Message::invocation("x", Arguments::default()),
           ),
    );
    let parser = Parser::new(r#"a = sequence { action() }"#).unwrap();
    expect(parser.arg(0),
           Argument::id_mes(
               "a",
               Message::lambda(TreeType::Sequence, Calls::new(vec![Call::invocation("action", Arguments::default())])),
           ),
    );
}

