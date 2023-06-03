use logos::Lexer;
use logos::Logos;
use crate::gol::ast::Number;

#[derive(Logos, Debug, Copy, Clone, PartialEq)]
#[logos(subpattern digit = r"[0-9]([0-9_]*[0-9])?")]
#[logos(subpattern letter = r"[a-zA-Z_]")]
#[logos(subpattern exp = r"[eE][+-]?[0-9]+")]
pub enum Token<'a> {
    #[regex(r"(?i)(?&letter)((?&letter)|(?&digit))*")]
    Id(&'a str),

    #[regex(r#""([^"\\]|\\t|\\u|\\n|\\")*""#,parse_qt_lit)]
    StringLit(&'a str),

    #[regex(r"-?(?&digit)", number)]
    #[regex(r"-?(?&digit)(?&exp)", number)]
    #[regex(r"-?(?&digit)?\.(?&digit)(?&exp)?[fFdD]?", float)]
    #[regex(r"0[bB][01][01]*", binary)]
    #[regex(r"-?0x[0-9a-f](([0-9a-f]|[_])*[0-9a-f])?", hex)]
    Digit(Number),

    #[token("(")]
    LParen,

    #[token(")")]
    RParen,

    #[token("{")]
    LBrace,

    #[token("}")]
    RBrace,

    #[token("=")]
    Assign,

    #[token("[")]
    LBrack,

    #[token("]")]
    RBrack,

    #[token(":")]
    Colon,


    #[token(";")]
    Semi,

    #[token(",")]
    Comma,

    #[token("false")]
    False,

    #[token("true")]
    True,

    #[token("array")]
    ArrayT,

    #[token("num")]
    NumT,
    #[token("object")]
    ObjectT,
    #[token("string")]
    StringT,
    #[token("bool")]
    BoolT,
    #[token("tree")]
    TreeT,

    #[regex(r"(?s)/\*.*\*/", logos::skip)]
    #[regex(r"//[^\r\n]*", logos::skip)]
    Comment,

    #[regex(r"[ \t\r\n\u000C\f]+", logos::skip)]
    Whitespace,

}

fn number<'a>(lex: &mut Lexer<'a, Token<'a>>) -> Option<Number> {
    lex.slice()
        .parse::<i64>()
        .map(|r| Number::Int(r))
        .ok()
}

fn float<'a>(lex: &mut Lexer<'a, Token<'a>>) ->  Option<Number> {
    lex.slice()
        .parse::<f64>()
        .map(|r| Number::Float(r))
        .ok()
}

fn binary<'a>(lex: &mut Lexer<'a, Token<'a>>) ->  Option<Number> {
    isize::from_str_radix(&lex.slice()[2..], 2)
        .map(Number::Binary)
        .ok()
}

fn hex<'a>(lex: &mut Lexer<'a, Token<'a>>) ->  Option<Number> {
    i64::from_str_radix(lex.slice().trim_start_matches("0x"), 16)
        .map(|r| Number::Hex(r))
        .ok()
}

fn parse_qt_lit<'a>(lexer: &mut Lexer<'a, Token<'a>>) ->  &'a str {
    let qt_lit: &str = lexer.slice();
    &qt_lit[1..qt_lit.len() - 1]
}

#[cfg(test)]
mod tests {
    use parsit::test::lexer_test as lt;
    use crate::gol::ast::Number;
    use crate::gol::lexer::Token;



    #[test]
    fn number() {
        lt::expect::<Token>(r#"1"#, vec![Token::Digit(Number::Int(1))]);
        lt::expect::<Token>(r#"1.1"#, vec![Token::Digit(Number::Float(1.1))]);
        lt::expect::<Token>(r#"1000000.000001"#, vec![Token::Digit(Number::Float(1000000.000001))]);
        lt::expect::<Token>(r#"1e-1"#, vec![Token::Digit(Number::Float(1000000.000001))]);

    }



}