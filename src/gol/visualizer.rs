use std::path::PathBuf;
use graphviz_rust::cmd::{CommandArg, Format};
use graphviz_rust::dot_generator::*;
use graphviz_rust::dot_structures::*;
use graphviz_rust::exec;
use graphviz_rust::printer::PrinterContext;
use crate::gol::GolError;
use crate::gol::project::Project;

struct Visualizer<'a> {
    project: &'a Project,
}

impl<'a> Visualizer<'a> {
    pub fn to_svg_file(mut self, path: String) -> Result<String, GolError> {
        let mut g = self.create_graph();

        exec(
            g,
            &mut PrinterContext::default(),
            vec![
                Format::Svg.into(),
                CommandArg::Output(path),
            ],
        ).map_err(|e| GolError::VisualizationError(e.to_string()))
    }

    fn create_graph(&self) -> Graph {
        let project = self.project;
        graph!(id!("id"))
    }
}

impl<'a> Visualizer<'a> {
    pub fn new(project: &'a Project) -> Self {
        Self { project }
    }
}


#[cfg(test)]
mod tests{
    use std::path::PathBuf;
    use crate::gol::project::Project;
    use crate::gol::visualizer::Visualizer;

    #[test]
    fn smoke(){
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