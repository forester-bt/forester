pub mod ast;
mod lexer;
mod tests;

use crate::tree::parser::ast::*;
use crate::tree::parser::lexer::Token;
use crate::tree::TreeError;
use ast::arg::{Argument, ArgumentRhs, Arguments, MesType, Param, Params};
use ast::call::{Call, Calls};
use ast::message::{Bool, Message, Number, StringLit};
use parsit::error::ParseError;
use parsit::parser::{EmptyToken, Parsit};
use parsit::step::Step;
use parsit::{seq, token, wrap};
use std::borrow::Cow;
use std::collections::HashMap;
use std::env::Args;
use std::fs;
use std::path::PathBuf;
use std::str::FromStr;

pub struct Parser<'a> {
    inner: Parsit<'a, Token>,
}

impl<'a> Parser<'a> {
    fn assign(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::Assign )
    }
    fn semi(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::Semi )
    }
    fn l_pr(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::LParen )
    }
    fn dot_dot(&self, pos: usize) -> Step<'a, EmptyToken> {
        token!(self.token(pos) => Token::DotDot )
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
        token!(self.token(pos) => Token::Id(v) => v.clone() )
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
        let r = |p| self.r_br(p);
        let comma = |p| self.comma(p);
        let message = |p| self.message(p);
        let elems = |p| seq!(p => message, comma,);
        let no_elems = vec![];

        wrap!(pos => l; elems or no_elems; r)
    }
    fn object(&'a self, pos: usize) -> Step<'a, HashMap<String, Message>> {
        let l = |p| self.l_brc(p);
        let r = |p| self.r_brc(p);
        let comma = |p| self.comma(p);

        let pair = |p| {
            self.str(p)
                .map(|l| l.0.to_string())
                .then_skip(|p| self.colon(p))
                .then_zip(|p| self.message(p))
        };

        let elems = |p| seq!(p => pair, comma,).map(|v| HashMap::from_iter(v));

        let def = HashMap::new();

        wrap!(pos => l; elems or def; r)
    }
    fn params(&self, pos: usize) -> Step<'a, Params> {
        let l = |p| self.l_pr(p);
        let r = |p| self.r_pr(p);
        let comma = |p| self.comma(p);

        let param = |p| {
            self.id(p)
                .then_skip(|p| self.colon(p))
                .then_zip(|p| self.mes_type(p))
                .map(|(name, tpe)| Param { name, tpe })
        };

        let elems = |p| seq!(p => param, comma,).map(|params| Params { params });
        let def = Params { params: vec![] };

        wrap!(pos => l; elems or def; r)
    }

    fn arg(&'a self, pos: usize) -> Step<'a, Argument> {
        let assign = |p| self.assign(p);
        let assigned = |p| self.id(p).then_skip(assign);

        let assign_id = |p| {
            assigned(p)
                .then_zip(|p| self.id(p).map(ArgumentRhs::Id))
                .map(|(a, b)| Argument::Assigned(a, b))
        };

        let assign_mes = |p| {
            assigned(p)
                .then_zip(|p| self.message(p).map(ArgumentRhs::Mes))
                .map(|(a, b)| Argument::Assigned(a, b))
        };
        let assign_call = |p| {
            assigned(p)
                .then_zip(|p| self.call(p).map(ArgumentRhs::Call))
                .map(|(a, b)| Argument::Assigned(a, b))
        };

        let mes = |p| {
            self.message(p)
                .map(ArgumentRhs::Mes)
                .map(Argument::Unassigned)
        };
        let id = |p| self.id(p).map(ArgumentRhs::Id).map(Argument::Unassigned);
        let call = |p| {
            self.call(p)
                .map(ArgumentRhs::Call)
                .map(Argument::Unassigned)
        };

        assign_mes(pos)
            .or_from(pos)
            .or(assign_call)
            .or(assign_id)
            .or(call)
            .or(mes)
            .or(id)
            .into()
    }

    fn args(&'a self, pos: usize) -> Step<'a, Arguments> {
        let l = |p| self.l_pr(p);
        let r = |p| self.r_pr(p);
        let comma = |p| self.comma(p);

        let arg = |p| self.arg(p);

        let elems = |p| seq!(p => arg, comma,).map(|args| Arguments { args });

        let def = Arguments { args: vec![] };

        wrap!(pos => l; elems or def; r)
    }
    fn tree_type(&self, pos: usize) -> Step<'a, TreeType> {
        self.id(pos)
            .flat_map(|p| TreeType::from_str(&p), |pe| Step::Fail(pos))
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
        self.str(pos)
            .map(Message::String)
            .or_from(pos)
            .or(|p| self.num(p).map(Message::Num))
            .or(|p| self.bool(p).map(Message::Bool))
            .or(|p| self.array(p).map(Message::Array))
            .or(|p| self.object(p).map(Message::Object))
            .into()
    }

    fn call_partial(&'a self, pos: usize) -> Step<'a, Key> {
        self.id(pos)
            .then_skip(|p| self.l_pr(p))
            .then_skip(|p| self.dot_dot(p))
            .then_skip(|p| self.r_pr(p))
    }

    fn call(&'a self, pos: usize) -> Step<'a, Call> {
        let inv = |p| {
            self.id(p)
                .then_zip(|p| self.args(p))
                .map(|(id, args)| Call::Invocation(id, args))
                .or_from(p)
                .or(|p| self.call_partial(p).map(Call::HoInvocation))
                .into()
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
            let l = |p| self.l_brc(p);
            let r = |p| self.r_brc(p);
            let calls = |p| self.inner.zero_or_more(p, |p| self.call(p));
            wrap!(p => l;calls;r).map(Calls::new)
        };

        calls(pos)
            .or_from(pos)
            .or(|p| self.call(p).map(|c| Calls { elems: vec![c] }))
            .into()
    }

    fn tree(&'a self, pos: usize) -> Step<'a, Tree> {
        self.tree_type(pos)
            .then_zip(|p| self.id(p))
            .then_or_default_zip(|p| self.params(p))
            .then_or_default_zip(|p| self.semi(p).map(|_| Calls::default()).or(|p| self.calls(p)))
            .map(|(((tpe, name), params), calls)| Tree {
                tpe,
                name,
                params,
                calls,
            })
    }

    fn import(&'a self, pos: usize) -> Step<'a, Import> {
        let l = |p| self.l_brc(p);
        let r = |p| self.r_brc(p);
        let comma = |p| self.comma(p);
        let name = |p| {
            self.id(p)
                .then_or_none_zip(|p| self.a_arr(p).then(|p| self.id(p)).or_none())
                .map(|(id, alias)| match alias {
                    None => ImportName::Id(id),
                    Some(a) => ImportName::Alias(id, a),
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
            .map(|(file, parts)| match parts {
                None => Import::file(file.0.as_str()),
                Some(names) => Import(file.0, names),
            })
    }

    fn file(&'a self, pos: usize) -> Step<'a, AstFile> {
        let entity = |p| {
            let entity: Step<FileEntity> = self
                .tree(p)
                .map(FileEntity::Tree)
                .or_from(p)
                .or(|p| self.import(p).map(FileEntity::Import))
                .into();
            entity
        };

        self.inner.zero_or_more(pos, entity).map(AstFile::new)
    }
}

impl<'a> Parser<'a> {
    pub fn new(src: &'a str) -> Result<Self, TreeError> {
        Ok(Parser {
            inner: Parsit::new(src)?,
        })
    }

    fn token(&self, pos: usize) -> Result<(&Token, usize), ParseError<'a>> {
        self.inner.token(pos)
    }

    pub fn parse(&'a self) -> Result<AstFile, TreeError> {
        let step: Step<AstFile> = self.inner.validate_eof(self.file(0));

        match step.clone() {
            Step::Success(file, pos) => Ok(file),
            Step::Fail(pos) => {
                let env = self.inner.env(&step);
                Err(TreeError::ParseError(format!(
                    "Parse error on the position: {pos} with the env: `{env}`"
                )))
            }
            Step::Error(ParseError::BadToken(t, _)) => {
                let env = self.inner.env(&step);
                Err(TreeError::ParseError(format!(
                    "Parse error the token: `{t}` is not recognized with the env: `{env}`"
                )))
            }
            Step::Error(ParseError::ExternalError(ext_t, pos)) => {
                let env = self.inner.env(&step);
                Err(TreeError::ParseError(format!(
                    "Parse error the token: `{ext_t}` on the pos:`{pos}` is not recognized with the env: `{env}`"
                )))
            }
            Step::Error(ParseError::FailedOnValidation(ext_t, pos)) => {
                let env = self.inner.env(&step);
                Err(TreeError::ParseError(format!(
                    "Parse error on the validation `{ext_t}` on the pos:`{pos}` is not recognized with the env: `{env}`"
                )))
            }
            Step::Error(ParseError::ReachedEOF(pos)) => {
                let env = self.inner.env(&step);
                Err(TreeError::ParseError(format!(
                    "Parse error on the pos:`{pos}` is reached eof with the env: `{env}`"
                )))
            }
            Step::Error(ParseError::UnreachedEOF(pos)) => {
                let env = self.inner.env(&step);
                Err(TreeError::ParseError(format!(
                    "Parse error on the pos:`{pos}` is unreached eof with the env: `{env}`"
                )))
            }
            Step::Error(err) => Err(TreeError::ParseError(err.to_string())),
        }
    }
}
