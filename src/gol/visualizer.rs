use std::path::PathBuf;
use graphviz_rust::cmd::{CommandArg, Format};
use graphviz_rust::dot_generator::*;
use graphviz_rust::dot_structures::*;
use graphviz_rust::exec;
use graphviz_rust::printer::PrinterContext;
use crate::gol::ast::Tree;
use crate::gol::GolError;
use crate::gol::project::Project;

fn tree_to_node(tree: &Tree) -> Stmt {
    let tree_type = format!("{}", *(&tree).tpe);
    let name = (&tree.name.0).to_string();
    let params:Vec<String> = tree.params.iter().map(|p|format!("{}:{:?}",p.name.0,p.tpe)).collect();



    node!()

}


struct Visualizer<'a> {
    project: &'a Project,
}

impl<'a> Visualizer<'a> {
    pub fn to_svg_file(&self, path: String) -> Result<String, GolError> {
        let mut g = create_graph(&self.project)?;

        exec(
            g,
            &mut PrinterContext::default(),
            vec![
                Format::Svg.into(),
                CommandArg::Output(path),
            ],
        ).map_err(|e| GolError::VisualizationError(e.to_string()))
    }
}


fn create_graph(project: &Project) -> Result<Graph, GolError> {
    let (main_file, root) = &project.main;
    let files = &project.files;

    let f = files.get(main_file).ok_or(GolError::CompileError(format!("no main file {}", main_file)))?;
    let root = f.definitions.get(root).ok_or(GolError::CompileError(format!("no root {} in {}", root, main_file)))?;


    Ok(graph!(id!("id")))
}

impl<'a> Visualizer<'a> {
    pub fn new(project: &'a Project) -> Self {
        Self { project }
    }
}


#[cfg(test)]
mod tests {
    use std::path::PathBuf;
    use crate::gol::project::Project;
    use crate::gol::visualizer::Visualizer;

    #[test]
    fn smoke() {
        let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let mut project_root = root.clone();
        let mut graph = root.clone();
        project_root.push("gol/tests/plain_project");
        let project = Project::build("main.gol".to_string(), project_root).unwrap();

        let v = Visualizer::new(&project);
        graph.push("gol/graphs/graph.svg");
        v.to_svg_file(graph.to_str().unwrap().to_string()).unwrap();
    }
}