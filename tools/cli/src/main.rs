use std::ffi::OsString;
use std::path::PathBuf;

use clap::{arg, value_parser, ArgMatches, Command};
use forester::runtime::action::Tick;
use forester::runtime::RtResult;
use forester::simulator::builder::SimulatorBuilder;
use forester::visualizer::Visualizer;

#[macro_use]
extern crate log;

fn cli() -> Command {
    Command::new("forest")
        .about("A console utility to interact with Forester")
        .subcommand_required(true)
        .arg_required_else_help(true)
        .version("0.1")
        .subcommand(
            Command::new("sim")
                .about(r#"Runs simulation. Expects a simulation profile"#)
                .arg(arg!(-p --profile <PATH> "a path to a sim profile"))
                .arg(arg!(-r --root <ROOT> "a path to a root folder. The <PWD> folder by default"))
                .arg(arg!(-m --main <MAIN> "a path to a main file. The 'main.tree' by default"))
                .arg(arg!(-t --tree <TREE> "a root in a main file. If there is only one root it takes by default"))
                .arg_required_else_help(true)
        )
}

fn buf(val: &str, relative: PathBuf) -> PathBuf {
    let path = PathBuf::from(val);
    if path.is_relative() {
        let mut full_path = relative;
        full_path.push(path);
        full_path
    } else {
        path
    }
}

fn sim(matches: &ArgMatches) {
    if let Some(p) = matches.get_one::<String>("profile") {
        let pwd = std::env::current_dir().expect("the current directory is not  presented");

        let root = match matches.get_one::<String>("root") {
            Some(root) => buf(root.as_str(), pwd),
            None => pwd,
        };

        let sim = buf(p, root.clone());
        let main_file = matches
            .get_one::<String>("main")
            .map(|v| v.to_string())
            .unwrap_or("main.tree".to_string());
        let main_tree = matches.get_one::<String>("tree");

        let mut sb = SimulatorBuilder::new();
        sb.profile(sim);
        sb.root(root);
        sb.main_file(main_file);
        if main_tree.is_some() {
            sb.main_tree(main_tree.unwrap().to_string())
        }

        match sb.build() {
            Ok(mut s) => match s.run() {
                Ok(r) => {
                    info!("the process is finished with the result: {:?}", r)
                }
                Err(err) => {
                    error!("the runtime error occured : {:?}", err)
                }
            },
            Err(err) => {
                error!("the building error occured: {:?}", err)
            }
        }
    } else {
        error!("the simulation profile is required")
    }
}

fn main() {
    env_logger::init();

    let matches = cli().get_matches();
    match matches.subcommand() {
        Some(("sim", args)) => {
            sim(args);
        }
        Some((e, _)) => {
            error!("the command '{e}' does not match the expected commands. ");
        }
        None => {
            unreachable!();
        }
    }
}
