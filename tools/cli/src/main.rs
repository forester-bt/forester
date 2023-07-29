use std::ffi::OsString;
use std::path::PathBuf;

use clap::{arg, value_parser, Arg, ArgAction, ArgMatches, Command};
use forester_rs::runtime::action::Tick;
use forester_rs::runtime::builder::builtin::BuilderBuiltInActions;
use forester_rs::runtime::builder::ForesterBuilder;
use forester_rs::runtime::RtResult;
use forester_rs::simulator::builder::SimulatorBuilder;
use forester_rs::tree::TreeError;
use forester_rs::visualizer::Visualizer;
use log::LevelFilter;

#[macro_use]
extern crate log;

fn cli() -> Command {
    Command::new("f-tree")
        .about("A console utility to interact with Forester")
        .subcommand_required(true)
        .arg_required_else_help(true)
        .version("0.1.9")
        .arg(
            Arg::new("debug")
                .short('d')
                .long("debug")
                .help("Print debug logs")
                .action(ArgAction::SetTrue)
        )
        .subcommand(Command::new("print-std").about("Print the list of std actions from 'import std::actions'"))
        .subcommand(
            Command::new("sim")
                .about(r#"Runs simulation. Expects a simulation profile"#)
                .arg(arg!(-p --profile <PATH> "a path to a sim profile, empty by default"))
                .arg(arg!(-r --root <ROOT> "a path to a root folder. The <PWD> folder by default"))
                .arg(arg!(-m --main <MAIN> "a path to a main file. The 'main.tree' by default"))
                .arg(arg!(-t --tree <TREE> "a root in a main file. If there is only one root it takes by default"))
        )
        .subcommand(
            Command::new("vis")
                .about(r#"Runs visualization. Output is in svg format."#)
                .arg(arg!(-o --output <OUTPUT> "a file for svg. If  no, the name from the main file will be taken."))
                .arg(arg!(-r --root <ROOT> "a path to a root folder. The <PWD> folder by default"))
                .arg(arg!(-m --main <MAIN> "a path to a main file. The 'main.tree' by default"))
                .arg(arg!(-t --tree <TREE> "a root in a main file. If there is only one root it takes by default"))

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
    let pwd = std::env::current_dir().expect("the current directory is presented");

    let root = match matches.get_one::<String>("root") {
        Some(root) => buf(root.as_str(), pwd),
        None => pwd,
    };

    let main_file = matches
        .get_one::<String>("main")
        .map(|v| v.to_string())
        .unwrap_or("main.tree".to_string());
    let main_tree = matches.get_one::<String>("tree");

    let mut sb = SimulatorBuilder::new();
    if let Some(p) = matches.get_one::<String>("profile") {
        let sim = buf(p, root.clone());
        sb.profile(sim);
    }
    sb.root(root.clone());
    let mut fb = ForesterBuilder::from_file_system();
    fb.main_file(main_file);
    fb.root(root);

    if main_tree.is_some() {
        fb.main_tree(main_tree.unwrap().to_string())
    }

    sb.forester_builder(fb);

    match sb.build() {
        Ok(mut s) => match s.run() {
            Ok(r) => {
                info!("the process is finished with the result: {:?}", r)
            }
            Err(err) => {
                error!("the runtime error occurred : {:?}", err)
            }
        },
        Err(err) => {
            error!("the building error occurred: {:?}", err)
        }
    }
}
fn viz(matches: &ArgMatches) {
    let pwd = std::env::current_dir().expect("the current directory is presented");

    let root = match matches.get_one::<String>("root") {
        Some(root) => buf(root.as_str(), pwd),
        None => pwd,
    };

    match Visualizer::visualize_to_file(
        root,
        matches.get_one::<String>("main"),
        matches.get_one::<String>("tree"),
        matches.get_one::<String>("output"),
    ) {
        Ok(_) => {
            info!("the result is successfully saved to the given file.")
        }
        Err(e) => {
            error!("the visualization is failed due to '{:?}'", e);
        }
    }
}
fn std() {
    let f = BuilderBuiltInActions::builtin_actions_file();
    info!("{f}");
}

fn main() {
    let matches = cli().get_matches();

    let mut log_builder = env_logger::builder();

    log_builder.is_test(false);
    if matches.get_flag("debug") {
        log_builder.filter_level(LevelFilter::max());
    }

    let _ = log_builder.try_init();

    match matches.subcommand() {
        Some(("sim", args)) => {
            sim(args);
        }
        Some(("vis", args)) => {
            viz(args);
        }
        Some(("print-std", _)) => {
            std();
        }
        Some((e, _)) => {
            error!("the command '{e}' does not match the expected commands. ");
        }
        None => {
            unreachable!();
        }
    }
}
