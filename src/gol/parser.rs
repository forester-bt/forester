mod tests;

use std::borrow::Cow;
use std::collections::HashMap;
use std::env::Args;
use std::fs;
use std::path::PathBuf;
use std::str::FromStr;
use parsit::error::ParseError;
use parsit::parser::{EmptyToken, Parsit};
use parsit::step::Step;
use parsit::{seq, token, wrap};
use crate::gol::ast::{Argument, Arguments, Bool, Call, Calls, AstFile, FileEntity, Key, Import, Message, MesType, Number, Param, Params, StringLit, Tree, TreeType, validate_lambda, ImportName};
use crate::gol::GolError;
use crate::gol::lexer::Token;

pub struct Parser<'a> {
    inner: Parsit<'a, Token>,
}

impl<'a> Parser<'a> {
    fn assign(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::Assign )
    }
    fn l_pr(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::LParen )
    }
    fn a_arr(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::AssignArr )
    }
    fn r_pr(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::RParen )
    }
    fn l_brc(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::LBrace )
    }
    fn r_brc(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::RBrace )
    }
    fn l_br(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::LBrack )
    }
    fn r_br(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::RBrack )
    }

    fn comma(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::Comma )
    }
    fn colon(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::Colon )
    }
    fn import_tk(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::Import )
    }

    fn id(&self, pos: usize) -> Step<'a, Key> {
        token!(self.token(pos) => Token::Id(v) => Key(v.clone()) )
    }
    fn str(&self, pos: usize) -> Step<'a, StringLit> {
        token!(self.token(pos) => Token::StringLit(v) => StringLit(v.clone()) )
    }
    fn num(&self, pos: usize) -> Step<'a, Number> {
        token!(self.token(pos) => Token::Digit(n) => n.clone() )
    }
    fn bool(&self, pos: usize) -> Step<'a, Bool> {
        token!(self.token(pos) =>
                Token::True => Bool::True ,
                Token::False => Bool::False
        )
    }


    fn array(&'a self, pos: usize) -> Step<'a, Vec<Message>> {
        let l = |p| self.l_br(p);
        let r = |p| { self.r_br(p) };
        let comma = |p| { self.comma(p) };
        let message = |p| { self.message(p) };
        let elems = |p| seq!(p => message, comma,);
        let no_elems = vec![];

        wrap!(pos => l; elems or no_elems; r)
    }
    fn object(&'a self, pos: usize) -> Step<'a, HashMap<String, Message>> {
        let l = |p| { self.l_brc(p) };
        let r = |p| { self.r_brc(p) };
        let comma = |p| { self.comma(p) };

        let pair = |p| {
            self.str(p)
                .map(|l| l.0.to_string())
                .then_skip(|p| self.colon(p))
                .then_zip(|p| self.message(p))
        };

        let elems = |p|
            seq!(p => pair, comma,)
                .map(|v| HashMap::from_iter(v));

        let def = HashMap::new();

        wrap!(pos => l; elems or def; r)
    }
    fn params(&self, pos: usize) -> Step<'a, Params> {
        let l = |p| { self.l_pr(p) };
        let r = |p| { self.r_pr(p) };
        let comma = |p| { self.comma(p) };

        let param = |p| {
            self.id(p)
                .then_skip(|p| self.colon(p))
                .then_zip(|p| self.mes_type(p))
                .map(|(name, tpe)| Param { name, tpe })
        };

        let elems = |p| {
            seq!(p => param, comma,)
                .map(|params| Params { params })
        };
        let def = Params { params: vec![] };

        wrap!(pos => l; elems or def; r)
    }

    fn arg(&'a self, pos: usize) -> Step<'a, Argument> {
        let assign = |p| { self.assign(p) };
        let assigned = |p| { self.id(p).then_skip(assign) };
        let assign_id = |p| {
            assigned(p)
                .then_zip(|p| self.id(p))
                .map(|(a, b)| Argument::AssignedId(a, b))
        };
        let assign_mes = |p| {
            assigned(p)
                .then_zip(|p| self.message(p))
                .map(|(a, b)| Argument::AssignedMes(a, b))
        };
        let mes = |p| self.message(p).map(Argument::Mes);
        let id = |p| self.id(p).map(Argument::Id);

        assign_mes(pos)
            .or_from(pos)
            .or(assign_id)
            .or(mes)
            .or(id)
            .into()
    }

    fn args(&'a self, pos: usize) -> Step<'a, Arguments> {
        let l = |p| { self.l_pr(p) };
        let r = |p| { self.r_pr(p) };
        let comma = |p| { self.comma(p) };

        let arg = |p| self.arg(p);

        let elems = |p| {
            seq!(p => arg, comma,)
                .map(|args| Arguments { args })
        };

        let def = Arguments { args: vec![] };

        wrap!(pos => l; elems or def; r)
    }
    fn tree_type(&self, pos: usize) -> Step<'a, TreeType> {
        self.id(pos)
            .flat_map(
                |p| TreeType::from_str(&p.0),
                |pe| Step::Fail(pos),
            )
    }

    fn mes_type(&self, pos: usize) -> Step<'a, MesType> {
        token!(self.token(pos) =>
                Token::StringT => MesType::String ,
                Token::NumT => MesType::Num,
                Token::ArrayT => MesType::Array,
                Token::BoolT => MesType::Bool,
                Token::TreeT => MesType::Tree,
                Token::ObjectT => MesType::Object
        )
    }

    fn message(&'a self, pos: usize) -> Step<'a, Message> {
        self.str(pos).map(Message::String)
            .or_from(pos)
            .or(|p| self.num(p).map(Message::Num))
            .or(|p| self.bool(p).map(Message::Bool))
            .or(|p| self.array(p).map(Message::Array))
            .or(|p| self.object(p).map(Message::Object))
            .or(|p| self.call(p).map(Message::Call))
            .into()
    }

    fn call(&'a self, pos: usize) -> Step<'a, Call> {
        let inv = |p| {
            self.id(p).then_zip(|p| self.args(p))
                .map(|(id, args)| Call::Invocation(id, args))
        };

        let anon = |p| {
            self.tree_type(p)
                .then_or_default_zip(|p| self.args(p))
                .then_zip(|p| self.calls(p))
                .validate(|((t, args), calls)| validate_lambda(t, args, calls))
                .map(|((t, args), calls)| {
                    if t.is_decorator() {
                        Call::decorator(t, args, calls.elems[0].clone())
                    } else {
                        Call::lambda(t, calls)
                    }
                })
        };

        anon(pos).or_from(pos).or(inv).into()
    }
    fn calls(&'a self, pos: usize) -> Step<'a, Calls> {
        let calls = |p| {
            let l = |p| { self.l_brc(p) };
            let r = |p| { self.r_brc(p) };
            let calls = |p| self.inner.zero_or_more(p, |p| self.call(p));
            wrap!(p => l;calls;r).map(Calls::new)
        };

        calls(pos)
            .or_from(pos)
            .or(|p| self.call(p)
                .map(|c| Calls { elems: vec![c] }))
            .into()
    }

    fn tree(&'a self, pos: usize) -> Step<'a, Tree> {
        self
            .tree_type(pos)
            .then_zip(|p| self.id(p))
            .then_or_default_zip(|p| self.params(p))
            .then_or_default_zip(|p| self.calls(p))
            .map(|(((tpe, name), params), calls)| Tree {
                tpe,
                name,
                params,
                calls,
            })
    }

    fn import(&'a self, pos: usize) -> Step<'a, Import> {
        let l = |p| { self.l_brc(p) };
        let r = |p| { self.r_brc(p) };
        let comma = |p| { self.comma(p) };
        let name = |p| {
            self.id(p).then_or_none_zip(|p|{
                self.a_arr(p).then(|p|self.id(p)).or_none()
            }).map(|(id,alias)|{
                match alias {
                    None => ImportName::Id(id.0),
                    Some(a) => ImportName::Alias(id.0,a.0),
                }
            })
        };
        let names = |p| seq!(p => name, comma,);


        let part = |p| {
            let def = vec![];
            wrap!(p => l;names or def; r ).or_none()
        };

        self.import_tk(pos)
            .then_zip(|p| self.str(p))
            .take_right()
            .then_or_none_zip(part)
            .map(|(file, parts)| {
                match parts {
                    None => Import::file(file.0.as_str()),
                    Some(names) => Import(file.0, names)
                }
            })
    }

    fn file(&'a self, pos: usize) -> Step<'a, AstFile> {
         let entity = |p|{
             let entity:Step<FileEntity> = self.tree(p).map(FileEntity::Tree)
                 .or_from(p)
                 .or(|p|self.import(p).map(FileEntity::Import))
                 .into();
             entity
         };

        self.inner.zero_or_more(pos,entity).map(AstFile::new)


    }
}

impl<'a> Parser<'a> {

    pub fn new(src: &'a str) -> Result<Self, GolError> {
        Ok(Parser {
            inner: Parsit::new(src)?,
        })
    }

    fn token(&self, pos: usize) -> Result<(&Token, usize), ParseError<'a>> {
        self.inner.token(pos)
    }

    pub fn parse(&'a self) -> Result<AstFile, ParseError<'a>> {
        self.inner.validate_eof(self.file(0)).into()
    }
}

