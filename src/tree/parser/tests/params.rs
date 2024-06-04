use crate::tree::parser::ast::arg::{MesType, Param, Params};

use crate::tree::parser::Parser;
use parsit::test::parser_test::expect;

#[test]
fn params() {
    let parser = Parser::new(r#"()"#).unwrap();
    expect(parser.params(0), Params::default());

    let parser = Parser::new(r#"(a:tree)"#).unwrap();
    expect(
        parser.params(0),
        Params::new(vec![Param::new("a", MesType::Tree)]),
    );

    let parser = Parser::new(r#"(a:tree, b:string)"#).unwrap();
    expect(
        parser.params(0),
        Params::new(vec![
            Param::new("a", MesType::Tree),
            Param::new("b", MesType::String),
        ]),
    );
    let parser = Parser::new(r#"(a:any)"#).unwrap();
    expect(
        parser.params(0),
        Params::new(vec![
            Param::new("a", MesType::Any),
        ]),
    );
}
