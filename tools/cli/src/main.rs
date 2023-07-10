use std::ffi::OsString;
use std::path::PathBuf;

use clap::{arg, Command};

fn cli() -> Command {
    Command::new("forest")
        .about("A console utility to interact with Forester")
        .subcommand_required(true)
        .arg_required_else_help(true)
        .subcommand(
            Command::new("sim")
                .about(r#"Runs simulation. Expects a simulation profile"#)
                .arg(arg!(--p --profile "a path to a sim profile").required(true))
                .arg(arg!(-r --root "a path to a root folder. The <PWD> folder by default").required(false))
                .arg(arg!(-m --main "a path to a main file. The 'main.tree' by default").required(false))
                .arg(arg!(-t --tree "a root in a main file. If there is only one root it takes by default").required(false))
                .arg_required_else_help(true),
        )
}

fn push_args() -> Vec<clap::Arg> {
    vec![arg!(-m --message <MESSAGE>)]
}

fn main() {
    let matches = cli().get_matches();
}
