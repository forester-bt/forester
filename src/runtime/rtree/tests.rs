#[cfg(test)]
mod tests {
    use crate::runtime::rtree::RuntimeTree;
    use crate::tree::project::Project;
    use std::path::PathBuf;

    fn tree(root_dir: &str, root_file: &str) -> RuntimeTree {
        let mut root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        root.push("tree/tests");
        root.push(root_dir);
        let project = Project::build(root_file.to_string(), root).unwrap();
        RuntimeTree::build(project).unwrap()
    }

    #[test]
    fn ho_op() {
        let tree = tree("units/ho", "main.tree");
        println!("{:?}", tree);
        assert_eq!(
            tree,
            RuntimeTree {
                root: 0,
                nodes: Default::default()
            }
        )
    }

    #[test]
    fn ho_tree() {
        let tree = tree("ho_tree", "main.tree");
        println!("{:?}", tree);
    }
    #[test]
    fn smoke() {
        let tree = tree("plain_project", "main.tree");
        println!("{:?}", tree);
    }
}
