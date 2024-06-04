use crate::converter::Converter;
use crate::converter::from_nav::FromNav2;
use crate::read_file;
use crate::runtime::rtree::RuntimeTree;
use crate::tests::{test_folder};
use crate::tree::project::Project;

#[test]
fn export_smoke() {
    let mut fb = test_folder("ros/nav/smoke");

    let project = Project::build("main.tree".to_string(), fb.clone()).unwrap();
    let tree = RuntimeTree::build(project).unwrap().tree;

    fb.push("test.xml");
    tree.to_ros_nav(fb.clone()).unwrap();

    let result = read_file(&fb).unwrap();

    assert_eq!(result, r#"<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <RateController hz="1">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <RecoveryNode number_of_retries="1">
              <PipelineSequence name="ComputePathWithFallback">
                <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                <ReactiveFallback name="ComputePathToPoseRecoveryFallback">
                  <GoalUpdated/>
                  <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
                </ReactiveFallback>
              </PipelineSequence>
            </RecoveryNode>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1">
          <PipelineSequence name="FollowPathWithFallback">
            <FollowPath path="{path}" controller_id="FollowPath"/>
            <ReactiveFallback name="FollowPathRecoveryFallback">
              <GoalUpdated/>
              <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
            </ReactiveFallback>
          </PipelineSequence>
        </RecoveryNode>
      </PipelineSequence>
    </RecoveryNode>
  </BehaviorTree>
</root>"#);
}
#[test]
fn import_smoke() {
    let mut fb = test_folder("ros/nav/smoke");
    fb.push("test.xml");
    let project = FromNav2::read_file(&fb).unwrap().convert().unwrap();
    println!("{:?}",project)


}
