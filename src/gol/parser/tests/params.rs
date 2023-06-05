use parsit::test::parser_test::expect;
use crate::gol::ast::*;
use crate::gol::parser::Parser;

#[test]
fn params() {
    let parser = Parser::new(r#"()"#).unwrap();
    expect(parser.params(0), Params::default());

    let parser = Parser::new(r#"(a:tree)"#).unwrap();
    expect(parser.params(0), Params::new(vec![Param::new("a",MesType::Tree)]));

    let parser = Parser::new(r#"(a:tree, b:string)"#).unwrap();
    expect(parser.params(0), Params::new(vec![
        Param::new("a",MesType::Tree),
        Param::new("b",MesType::String),
    ]
    ));

}



