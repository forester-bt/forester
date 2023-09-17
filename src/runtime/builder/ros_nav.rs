use std::collections::HashMap;
use std::fmt::{Display, Formatter};
use itertools::Itertools;
use crate::runtime::action::{Action, ActionName};
use crate::runtime::action::builtin::ReturnResult;
use crate::runtime::{RtResult, RuntimeError};
use crate::tree::parser::ast::arg::{MesType, Param};
use crate::tree::parser::ast::message::Message;


pub(super) fn action_impl(action: &ActionName) -> RtResult<Action> {
    match action.as_str() {
        _ if ros_actions().contains_key(action) => Ok(Action::sync(ReturnResult::success())),

        _ => Err(RuntimeError::UnImplementedAction(format!(
            "action {action} is absent in the library"
        ))),
    }
}

pub fn ros_actions_file() -> String {
    let actions = ros_actions().iter().map(|(_, v)| v.to_string()).join("\n\n");
    format!(r#"// Ros specific actions and decorators.
// The actions are accessible using the import 'import "ros::nav2"'

// --- Control nodes ---

// PipelineSequence - Sequence of actions that are executed in a pipeline fashion.
// In Forester, it is represented as a sequence

// RoundRobin - Sequence of actions that are executed in a round robin fashion.
// In Forester, it is represented as a fallback

// RecoveryNode
// The RecoveryNode is a control flow node with two children.
// It returns SUCCESS if and only if the first child returns SUCCESS.
// The second child will be executed only if the first child returns FAILURE.
// If the second child SUCCEEDS, then the first child will be executed again.
// The user can specify how many times the recovery actions should be taken before returning FAILURE.
// In nav2, the RecoveryNode is included in Behavior Trees to implement recovery actions upon failures.
// In Forester, it is represented as a decorator retry
// Also there is a specific node RecoveryNode that is used in the nav2 Behavior Trees.


// --- Actions ---

{actions}

"#)
}


pub fn ros_actions() -> HashMap<String, RosAction> {
    HashMap::from_iter(vec![
        kv(RosAction::new(
            "Wait".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("wait_duration", MesType::Num), Message::float(1.0)),
                RosParam::Input(Param::new("server_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
            ],
            r#"
// Invokes the Wait ROS 2 action server, which is implemented by the nav2_behaviors module.
// This action is used in nav2 Behavior Trees as a recovery behavior.
// <Wait wait_duration="1.0" server_name="wait_server" server_timeout="10"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "Spin".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("spin_dist", MesType::Num), Message::float(1.57)),
                RosParam::InputWithDefault(Param::new("time_allowance", MesType::Num), Message::float(10.0)),
                RosParam::Input(Param::new("server_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
                RosParam::InputWithDefault(Param::new("is_recovery", MesType::Bool), Message::bool(true)),
                RosParam::Output(Param::new("error_code_id", MesType::Num)),
            ],
            r#"
// Invokes the Spin ROS 2 action server, which is implemented by the nav2_behaviors module. It performs an in-place rotation by a given angle.
// This action is used in nav2 Behavior Trees as a recovery behavior.
// <Spin spin_dist="1.57" server_name="spin" server_timeout="10" is_recovery="true" error_code_id="{spin_error_code}"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "BackUp".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("backup_dist", MesType::Num), Message::float(-0.15)),
                RosParam::InputWithDefault(Param::new("backup_speed", MesType::Num), Message::float(0.025)),
                RosParam::InputWithDefault(Param::new("time_allowance", MesType::Num), Message::float(10.0)),
                RosParam::Input(Param::new("server_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
                RosParam::Output(Param::new("error_code_id", MesType::Num)),
            ],
            r#"
// Invokes the BackUp ROS 2 action server, which causes the robot to back up by a specific displacement.
// It performs an linear translation by a given distance. This is used in nav2 Behavior Trees as a recovery behavior.
// The nav2_behaviors module implements the BackUp action server.
// <BackUp backup_dist="-0.2" backup_speed="0.05" server_name="backup_server" server_timeout="10" error_code_id="{backup_error_code}"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "DriveOnHeading".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("dist_to_travel", MesType::Num), Message::float(0.15)),
                RosParam::InputWithDefault(Param::new("speed", MesType::Num), Message::float(0.025)),
                RosParam::InputWithDefault(Param::new("time_allowance", MesType::Num), Message::float(10.0)),
                RosParam::Input(Param::new("server_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
                RosParam::Output(Param::new("error_code_id", MesType::Num)),
            ],
            r#"
// Invokes the DriveOnHeading ROS 2 action server, which causes the robot to drive on the current heading by a specific displacement.
// It performs a linear translation by a given distance.
// The nav2_behaviors module implements the DriveOnHeading action server.
// <DriveOnHeading dist_to_travel="0.2" speed="0.05" server_name="backup_server" server_timeout="10" error_code_id="{drive_on_heading_error_code}"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "AssistedTeleop".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("is_recovery", MesType::Bool), Message::bool(false)),
                RosParam::InputWithDefault(Param::new("time_allowance", MesType::Num), Message::float(10.0)),
                RosParam::Input(Param::new("server_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
                RosParam::Output(Param::new("error_code_id", MesType::Num)),
            ],
            r#"
// Invokes the AssistedTeleop ROS 2 action server, which filters teleop twist commands to prevent collisions.
// This is used in nav2 Behavior Trees as a recovery behavior or a regular behavior.
// The nav2_behaviors module implements the AssistedTeleop action server.
// <AssistedTeleop is_recovery="false" server_name="assisted_teleop_server" server_timeout="10" error_code_id="{assisted_teleop_error_code}"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "ComputePathToPose".to_string(),
            vec![
                RosParam::Input(Param::new("start", MesType::Object)),
                RosParam::Input(Param::new("goal", MesType::Object)),
                RosParam::Input(Param::new("planner_id", MesType::String)),
                RosParam::Input(Param::new("server_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
                RosParam::Output(Param::new("path", MesType::Object)),
                RosParam::Output(Param::new("error_code_id", MesType::Num)),
            ],
            r#"
// Invokes the ComputePathToPose ROS 2 action server, which is implemented by the nav2_planner module.
// The server address can be remapped using the server_name input port.
// <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased" server_name="ComputePathToPose" server_timeout="10" error_code_id="{compute_path_error_code}"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "FollowPath".to_string(),
            vec![
                RosParam::Input(Param::new("path", MesType::String)),
                RosParam::Input(Param::new("controller_id", MesType::String)),
                RosParam::Input(Param::new("goal_checker_id", MesType::String)),
                RosParam::Input(Param::new("server_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
                RosParam::Output(Param::new("error_code_id", MesType::Num)),
            ],
            r#"
// Invokes the FollowPath ROS 2 action server, which is implemented by the controller plugin modules loaded.
// The server address can be remapped using the server_name input port.
// <FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="precise_goal_checker" server_name="FollowPath" server_timeout="10" error_code_id="{follow_path_error_code}"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "NavigateToPose".to_string(),
            vec![
                RosParam::Input(Param::new("goal", MesType::Object)),
                RosParam::Input(Param::new("server_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
                RosParam::Input(Param::new("behavior_tree", MesType::String)),
                RosParam::Output(Param::new("error_code_id", MesType::Num)),
            ],
            r#"
// Invokes the FollowPath ROS 2 action server, which is implemented by the controller plugin modules loaded.
// The server address can be remapped using the server_name input port.
// Note: For the behavior tree absolute path: If none is specified, NavigateToPose action server uses a default behavior tree.
// <FollowPath path="{path}" controller_id="FollowPath" goal_checker_id="precise_goal_checker" server_name="FollowPath" server_timeout="10" error_code_id="{follow_path_error_code}"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "ClearEntireCostmap".to_string(),
            vec![
                RosParam::Input(Param::new("service_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
            ],
            r#"
// Action to call a costmap clearing server.
// <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "ClearCostmapExceptRegion".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("reset_distance", MesType::Num), Message::float(1.0)),
                RosParam::Input(Param::new("server_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
            ],
            r#"
// Action to call a costmap clearing except region server.
// <ClearCostmapExceptRegion name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_except_local_costmap"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "ClearCostmapAroundRobot".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("reset_distance", MesType::Num), Message::float(1.0)),
                RosParam::Input(Param::new("server_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
            ],
            r#"
// Action to call a costmap clearing around robot server.
// <ClearCostmapAroundRobot name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_around_local_costmap"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "ReinitializeGlobalLocalization".to_string(),
            vec![
                RosParam::Input(Param::new("server_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
            ],
            r#"
// Used to trigger global relocalization using AMCL in case of severe delocalization or kidnapped robot problem.
// <ReinitializeGlobalLocalization service_name="reinitialize_global_localization"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "TruncatePath".to_string(),
            vec![
                RosParam::Input(Param::new("input_path", MesType::Object)),
                RosParam::InputWithDefault(Param::new("distance", MesType::Num), Message::float(1.0)),
                RosParam::Output(Param::new("output_path", MesType::Object)),
            ],
            r#"
// A custom control node, which modifies a path making it shorter.
// It removes parts of the path closer than a distance to the goal pose.
// The resulting last pose of the path orientates the robot to the original goal pose.
// <TruncatePath distance="1.0" input_path="{path}" output_path="{truncated_path}"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "TruncatePathLocal".to_string(),
            vec![
                RosParam::Input(Param::new("input_path", MesType::Object)),
                RosParam::InputWithDefault(Param::new("distance_forward", MesType::Num), Message::float(8.0)),
                RosParam::InputWithDefault(Param::new("distance_backward", MesType::Num), Message::float(4.0)),
                RosParam::InputWithDefault(Param::new("robot_frame", MesType::String), Message::str("base_link")),
                RosParam::InputWithDefault(Param::new("transform_tolerance", MesType::Num), Message::float(0.2)),
                RosParam::Input(Param::new("pose", MesType::Object)),
                RosParam::InputWithDefault(Param::new("angular_distance_weight", MesType::Num), Message::float(0.0)),
                RosParam::InputWithDefault(Param::new("max_robot_pose_search_dist", MesType::Num), Message::float(f64::MAX)),
                RosParam::Output(Param::new("output_path", MesType::Object)),
            ],
            r#"
// A custom control node, which modifies a path making it shorter.
// It removes parts of the path which are more distant than specified forward/backward distance around robot
// <TruncatePathLocal input_path="{path}" output_path="{path_local}" distance_forward="3.5" distance_backward="2.0" robot_frame="base_link"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "PlannerSelector".to_string(),
            vec![
                RosParam::Input(Param::new("topic_name", MesType::String)),
                RosParam::Input(Param::new("default_planner", MesType::String)),
                RosParam::Output(Param::new("selected_planner", MesType::String)),
            ],
            r#"
// It is used to select the planner that will be used by the planner server.
// It subscribes to the planner_selector topic to receive command messages with the name of the planner to be used.
// It is commonly used before of the ComputePathToPoseAction.
// The selected_planner output port is passed to planner_id input port of the ComputePathToPoseAction.
// If none is provided on the topic, the default_planner is used.
// <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "ControllerSelector".to_string(),
            vec![
                RosParam::Input(Param::new("topic_name", MesType::String)),
                RosParam::Input(Param::new("default_controller", MesType::String)),
                RosParam::Output(Param::new("selected_controller", MesType::String)),
            ],
            r#"
// It is used to select the Controller that will be used by the Controller server.
// It subscribes to the controller_selector topic to receive command messages with the name of the Controller to be used.
// It is commonly used before of the FollowPathAction. The selected_controller output port is passed to controller_id input port of the FollowPathAction.
// If none is provided on the topic, the default_controller is used.
// <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "SmootherSelector".to_string(),
            vec![
                RosParam::Input(Param::new("topic_name", MesType::String)),
                RosParam::Input(Param::new("default_smoother", MesType::String)),
                RosParam::Output(Param::new("selected_smoother", MesType::String)),
            ],
            r#"
// It is used to select the Smoother that will be used by the Smoother server.
// It subscribes to the smoother_selector topic to receive command messages with the name of the Smoother to be used.
// It is commonly used before of the FollowPathAction. If none is provided on the topic, the default_smoother is used.
// Any publisher to this topic needs to be configured with some QoS defined as reliable and transient local.
// <SmootherSelector selected_smoother="{selected_smoother}" default_smoother="SimpleSmoother" topic_name="smoother_selector"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "GoalCheckerSelector".to_string(),
            vec![
                RosParam::Input(Param::new("topic_name", MesType::String)),
                RosParam::Input(Param::new("default_goal_checker", MesType::String)),
                RosParam::Output(Param::new("selected_goal_checker", MesType::String)),
            ],
            r#"
// It is used to select the GoalChecker that will be used by the goal_checker server.
// It subscribes to the goal_checker_selector topic to receive command messages with the name of the GoalChecker to be used.
// It is commonly used before of the FollowPathAction.
// The selected_goal_checker output port is passed to goal_checker_id input port of the FollowPathAction.
// If none is provided on the topic, the default_goal_checker is used.
// Any publisher to this topic needs to be configured with some QoS defined as reliable and transient local.
// <GoalCheckerSelector selected_goal_checker="{selected_goal_checker}" default_goal_checker="precise_goal_checker" topic_name="goal_checker_selector"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "NavigateThroughPoses".to_string(),
            vec![
                RosParam::Input(Param::new("goals", MesType::Object)),
                RosParam::Input(Param::new("server_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
                RosParam::Input(Param::new("behavior_tree", MesType::String)),
                RosParam::Output(Param::new("error_code_id", MesType::Num)),
            ],
            r#"
// Invokes the NavigateThroughPoses ROS 2 action server, which is implemented by the bt_navigator module.
// <NavigateThroughPoses goals="{goals}" server_name="NavigateThroughPoses" server_timeout="10" error_code_id="{navigate_through_poses_error_code}"
//                       behavior_tree="<some-path>/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "ComputePathThroughPoses".to_string(),
            vec![
                RosParam::Input(Param::new("start", MesType::Object)),
                RosParam::Input(Param::new("goals", MesType::Object)),
                RosParam::Input(Param::new("planner_id", MesType::String)),
                RosParam::Input(Param::new("server_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
                RosParam::Output(Param::new("path", MesType::Object)),
                RosParam::Output(Param::new("error_code_id", MesType::Num)),
            ],
            r#"
// Invokes the ComputePathThroughPoses ROS 2 action server, which is implemented by the nav2_planner module.
// The server address can be remapped using the server_name input port.
// <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="GridBased" server_name="ComputePathThroughPoses" server_timeout="10" error_code_id="{compute_path_error_code}"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "RemovePassedGoals".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("radius", MesType::Num), Message::float(0.5)),
                RosParam::InputWithDefault(Param::new("global_frame", MesType::String), Message::str("map")),
                RosParam::InputWithDefault(Param::new("robot_base_frame", MesType::String), Message::str("base_link")),
                RosParam::Input(Param::new("input_goals", MesType::Object)),
                RosParam::Output(Param::new("output_goals", MesType::Object)),
            ],
            r#"
// Looks over the input port goals and removes any point that the robot is in close proximity to or has recently passed.
// This is used to cull goal points that have been passed from ComputePathToPoses to enable replanning to only the current task goals.
// <RemovePassedGoals radius="0.6" input_goals="{goals}" output_goals="{goals}"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "CancelControl".to_string(),
            vec![
                RosParam::Input(Param::new("service_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
            ],
            r#"
// Used to cancel the goals given to the controllers’ action server.
// The server address can be remapped using the server_name input port.
// <CancelControl server_name="FollowPath" server_timeout="10"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "CancelBackUp".to_string(),
            vec![
                RosParam::Input(Param::new("service_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
            ],
            r#"
// Used to cancel the backup action that is part of the behavior server.
// The server address can be remapped using the server_name input port.
// <CancelBackUp server_name="BackUp" server_timeout="10"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "CancelSpin".to_string(),
            vec![
                RosParam::Input(Param::new("service_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
            ],
            r#"
// Used to cancel the spin action that is part of the behavior server.
// The server address can be remapped using the server_name input port.
// <CancelSpin server_name="Spin" server_timeout="10"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "CancelWait".to_string(),
            vec![
                RosParam::Input(Param::new("service_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
            ],
            r#"
// Used to cancel the wait action that is part of the behavior server.
// The server address can be remapped using the server_name input port.
// <CancelWait server_name="Spin" server_timeout="10"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "CancelDriveOnHeading".to_string(),
            vec![
                RosParam::Input(Param::new("service_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
            ],
            r#"
// Used to cancel the drive on heading action that is part of the behavior server.
// The server address can be remapped using the server_name input port.
// <CancelDriveOnHeading server_name="Spin" server_timeout="10"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "CancelAssistedTeleop".to_string(),
            vec![
                RosParam::Input(Param::new("service_name", MesType::String)),
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
            ],
            r#"
// Used to cancel the AssistedTeleop action that is part of the behavior server.
// The server address can be remapped using the server_name input port.
// <CancelAssistedTeleop server_name="Spin" server_timeout="10"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "SmoothPath".to_string(),
            vec![
                RosParam::Input(Param::new("unsmoothed_path", MesType::String)),
                RosParam::Input(Param::new("smoother_id", MesType::String)),
                RosParam::InputWithDefault(Param::new("max_smoothing_duration", MesType::Num), Message::float(3.0)),
                RosParam::InputWithDefault(Param::new("check_for_collisions", MesType::Bool), Message::bool(false)),
                RosParam::Output(Param::new("smoothed_path", MesType::String)),
                RosParam::Output(Param::new("smoothing_duration", MesType::Num)),
                RosParam::Output(Param::new("was_completed", MesType::Bool)),
                RosParam::Output(Param::new("error_code_id", MesType::Num)),
            ],
            r#"
// Invokes the SmoothPath action API in the smoother server to smooth a given path plan.
// <SmoothPath unsmoothed_path="{path}" smoothed_path="{path}" max_smoothing_duration="3.0"
//      smoother_id="simple_smoother" check_for_collisions="false"
//      smoothing_duration="{smoothing_duration_used}" was_completed="{smoothing_completed}"
//      error_code_id="{smoothing_path_error_code}"/>
|params_doc|
impl |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "GoalReached".to_string(),
            vec![
                RosParam::Input(Param::new("goal", MesType::String)),
                RosParam::InputWithDefault(Param::new("global_frame", MesType::String), Message::str("map")),
                RosParam::InputWithDefault(Param::new("robot_base_frame", MesType::String), Message::str("base_link")),
            ],
            r#"
// Checks the distance to the goal, if the distance to goal is less than the pre-defined threshold,
// the tree returns SUCCESS, otherwise it returns FAILURE.
// <GoalReached goal="{goal}" global_frame="map" robot_base_frame="base_link"/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "TransformAvailable".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("child", MesType::String), Message::str("")),
                RosParam::InputWithDefault(Param::new("parent", MesType::String), Message::str("")),
            ],
            r#"
// Checks if a TF transform is available. Returns failure if it cannot be found.
// Once found, it will always return success. Useful for initial condition checks.
// <TransformAvailable parent="odom" child="base_link"/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "DistanceTraveled".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("distance", MesType::Num), Message::float(1.0)),
                RosParam::InputWithDefault(Param::new("global_frame", MesType::String), Message::str("map")),
                RosParam::InputWithDefault(Param::new("robot_base_frame", MesType::String), Message::str("base_link")),
            ],
            r#"
// Node that returns success when a configurable distance has been traveled.
// <DistanceTraveled distance="0.8" global_frame="map" robot_base_frame="base_link"/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "GoalUpdated".to_string(),
            vec![],
            r#"
// Checks if the global navigation goal has changed in the blackboard.
// Returns failure if the goal is the same, if it changes, it returns success.
// <GoalUpdated/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "GloballyUpdatedGoal".to_string(),
            vec![],
            r#"
// Checks if the global navigation goal has changed in the blackboard.
// Returns failure if the goal is the same, if it changes, it returns success.
//
// This node differs from the GoalUpdated by retaining the state of the current goal/goals
// throughout each tick of the BehaviorTree such that it will update on any “global” change to the goal.
// <GlobalUpdatedGoal/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "InitialPoseReceived".to_string(),
            vec![],
            r#"
// Node that returns success when the initial pose is sent to AMCL via /initial_pose`.
// <InitialPoseReceived/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "IsStuck".to_string(),
            vec![],
            r#"
// Determines if the robot is not progressing towards the goal.
// If the robot is stuck and not progressing, the condition returns SUCCESS, otherwise it returns FAILURE.
// <IsStuck/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "TimeExpired".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("seconds", MesType::Num), Message::float(1.0)),
            ],
            r#"
// Node that returns success when a time duration has passed
// <TimeExpired seconds="1.0"/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "IsBatteryLow".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("min_battery", MesType::Num), Message::float(0.5)),
                RosParam::InputWithDefault(Param::new("is_voltage", MesType::Bool), Message::bool(false)),
                RosParam::Input(Param::new("battery_topic", MesType::String)),
            ],
            r#"
// Checks if battery is low by subscribing to a sensor_msgs/BatteryState topic and checking if battery percentage/voltage is below a specified minimum value. By default percentage (in range 0 to 1) is used to check for low battery.
// Set the is_voltage parameter to true to use voltage.
// Returns SUCCESS when battery percentage/voltage is lower than the specified value, FAILURE otherwise.
// <IsBatteryLow min_battery="0.5" battery_topic="/battery_status" is_voltage="false"/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "IsPathValid".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("server_timeout", MesType::Num), Message::float(10.0)),
                RosParam::Input(Param::new("path", MesType::Object)),
            ],
            r#"
// Checks to see if the global path is valid. If there is a obstacle along the path,
// the condition returns FAILURE, otherwise it returns SUCCESS.
// <IsPathValid server_timeout="10" path="{path}"/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "AreErrorCodesPresent".to_string(),
            vec![
                RosParam::Input(Param::new("error_code", MesType::Num)),
                RosParam::Input(Param::new("error_codes_to_check", MesType::Num)),
            ],
            r#"
// Checks the if the provided error code matches any error code within a set.
//
// If the active error code is a match, the node returns SUCCESS. Otherwise, it returns FAILURE.
// Error codes to check are defined in another port:
// <AreErrorCodesPresent error_code="{error_code}" error_codes_to_check="{error_codes_to_check}"/>
//
// Error codes to check are defined to be 101, 107 and 119.
// <AreErrorCodesPresent error_code="{error_code}" error_codes_to_check="101,107,119"/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "WouldAControllerRecoveryHelp".to_string(),
            vec![
                RosParam::Input(Param::new("error_code", MesType::Num)),
            ],
            r#"
// Checks if the active controller server error code is UNKNOWN, PATIENCE_EXCEEDED, FAILED_TO_MAKE_PROGRESS, or NO_VALID_CONTROL.
//
// If the active error code is a match, the node returns SUCCESS. Otherwise, it returns FAILURE.
// <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "WouldAPlannerRecoveryHelp".to_string(),
            vec![
                RosParam::Input(Param::new("error_code", MesType::Num)),
            ],
            r#"
// Checks if the active controller server error code is UNKNOWN, NO_VALID_CONTROL, or TIMEOUT.
//
// If the active error code is a match, the node returns SUCCESS. Otherwise, it returns FAILURE.
// <WouldAPlannerRecoveryHelp  error_code="{compute_path_to_pose_error_code}"/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "WouldASmootherRecoveryHelp".to_string(),
            vec![
                RosParam::Input(Param::new("error_code", MesType::Num)),
            ],
            r#"
// Checks if the active controller server error code is UNKNOWN, TIMEOUT, FAILED_TO_SMOOTH_PATH,
// or SMOOTHED_PATH_IN_COLLISION.
//
// If the active error code is a match, the node returns SUCCESS. Otherwise, it returns FAILURE.
// <WouldASmootherRecoveryHelp   error_code="{smoother_error_code}"/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "IsBatteryCharging".to_string(),
            vec![
                RosParam::Input(Param::new("battery_topic", MesType::String)),
            ],
            r#"
// Checks if the battery is charging by subscribing
// to a sensor_msgs/BatteryState topic and checking
// if the power_supply_status is POWER_SUPPLY_STATUS_CHARGING.
// Returns SUCCESS in that case, FAILURE otherwise.
// <IsBatteryCharging battery_topic="/battery_status"/>
|params_doc|
cond |header|;
"#.to_string(),
        )),
        kv(RosAction::new(
            "RateController".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("hz", MesType::Num), Message::float(10.0)),
                RosParam::Input(Param::new("sub", MesType::Tree)),
            ],
            r#"
// A node that throttles the tick rate for its child. The tick rate can be supplied to the node as a parameter.
// The node returns RUNNING when it is not ticking its child.
// Currently, in the navigation stack, the RateController is used to adjust the rate
// at which the ComputePathToPose and GoalReached nodes are ticked.
// <RateController hz="1.0">
//     <!--Add tree components here--->
// </RateController>
|params_doc|
sequence |header| sub(..)
"#.to_string(),
        )),
        kv(RosAction::new(
            "DistanceController".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("distance", MesType::Num), Message::float(1.0)),
                RosParam::InputWithDefault(Param::new("global_frame", MesType::String), Message::str("map")),
                RosParam::InputWithDefault(Param::new("robot_base_frame", MesType::String), Message::str("base_link")),
                RosParam::Input(Param::new("sub", MesType::Tree)),
            ],
            r#"
// A node that controls the tick rate for its child based on the distance traveled.
// The distance to be traveled before replanning can be supplied to the node as a parameter.
// The node returns RUNNING when it is not ticking its child.
// Currently, in the navigation stack, the DistanceController is used to adjust the rate
// at which the ComputePathToPose and GoalReached nodes are ticked.
// <DistanceController distance="0.5" global_frame="map" robot_base_frame="base_link">
//   <!--Add tree components here--->
// </DistanceController>
|params_doc|
sequence |header| sub(..)
"#.to_string(),
        )),
        kv(RosAction::new(
            "SpeedController".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("min_rate", MesType::Num), Message::float(0.1)),
                RosParam::InputWithDefault(Param::new("max_rate", MesType::Num), Message::float(1.0)),
                RosParam::InputWithDefault(Param::new("min_speed", MesType::Num), Message::float(0.0)),
                RosParam::InputWithDefault(Param::new("max_speed", MesType::Num), Message::float(0.5)),
                RosParam::InputWithDefault(Param::new("filter_duration", MesType::Num), Message::float(0.3)),
                RosParam::Input(Param::new("sub", MesType::Tree)),
            ],
            r#"
// A node that controls the tick rate for its child based on current robot speed.
// The maximum and minimum replanning rates can be supplied to the node as parameters along with maximum and minimum speed.
// The node returns RUNNING when it is not ticking its child.
// Currently, in the navigation stack,
// the SpeedController is used to adjust the rate at which the ComputePathToPose and GoalReached nodes are ticked.
// <SpeedController min_rate="0.1" max_rate="1.0" min_speed="0.0" max_speed="0.5" filter_duration="0.3">
//   <!--Add tree components here--->
// </SpeedController>
|params_doc|
sequence |header| sub(..)
"#.to_string(),
        )),
        kv(RosAction::new(
            "GoalUpdater".to_string(),
            vec![
                RosParam::Input(Param::new("input_goal", MesType::Object)),
                RosParam::Input(Param::new("output_goal", MesType::Object)),
                RosParam::Input(Param::new("sub", MesType::Tree)),
            ],
            r#"
// A custom control node, which updates the goal pose.
// It subscribes to a topic in which it can receive an updated goal pose to use instead of the one commanded in action.
// It is useful for dynamic object following tasks.
// <GoalUpdater input_goal="{goal}" output_goal="{updated_goal}">
//   <!--Add tree components here--->
// </GoalUpdater>
|params_doc|
sequence |header| sub(..)
"#.to_string(),
        )),
        kv(RosAction::new(
            "PathLongerOnApproach".to_string(),
            vec![
                RosParam::Input(Param::new("path", MesType::Object)),
                RosParam::InputWithDefault(Param::new("prox_len", MesType::Num), Message::float(3.0)),
                RosParam::InputWithDefault(Param::new("length_factor", MesType::Num), Message::float(2.0)),
                RosParam::Input(Param::new("sub", MesType::Tree)),
            ],
            r#"
// This node checks if the newly generated global path is significantly larger
// than the old global path in the user-defined robot’s goal proximity
// and triggers their corresponding children.
// This allows users to enact special behaviors before a robot attempts
// to execute a path significantly longer than the prior path
// when close to a goal (e.g. going around an dynamic obstacle
// that may just need a few seconds to move out of the way).
// <PathLongerOnApproach path="{path}" prox_len="3.0" length_factor="2.0">
//   <!--Add tree components here--->
// </PathLongerOnApproach>
|params_doc|
sequence |header| sub(..)
"#.to_string(),
        )),
        kv(RosAction::new(
            "RecoveryNode".to_string(),
            vec![
                RosParam::InputWithDefault(Param::new("number_of_retries", MesType::Num), Message::int(1)),
                RosParam::Input(Param::new("sub", MesType::Tree)),
            ],
            r#"
// The RecoveryNode is a control flow node with two children.
// It returns SUCCESS if and only if the first child returns SUCCESS.
// The second child will be executed only if the first child returns FAILURE.
// If the second child SUCCEEDS, then the first child will be executed again.
// The user can specify how many times the recovery actions should be taken before returning FAILURE.
// In nav2, the RecoveryNode is included in Behavior Trees to implement recovery actions upon failures.
// <RecoveryNode number_of_retries="1">
//     <!--Add tree components here--->
// </RecoveryNode>
|params_doc|
sequence |header| sub(..)
"#.to_string(),
        )),
        kv(RosAction::new(
            "SingleTrigger".to_string(),
            vec![
                RosParam::Input(Param::new("sub", MesType::Tree)),
            ],
            r#"
// This node triggers its child only once and returns FAILURE for every succeeding tick.
// <SingleTrigger>
//   <!--Add tree components here--->
// </SingleTrigger>
|params_doc|
sequence |header| sub(..)
"#.to_string(),
        )),
    ])
}

pub fn find_ros_action(name: &str) -> Option<RosAction> {
    ros_actions().get(name).cloned()
}

fn kv(a: RosAction) -> (String, RosAction) {
    (a.name.clone(), a)
}

#[derive(Debug, Clone)]
pub struct RosAction {
    pub name: String,
    pub params: Vec<RosParam>,
    pub show: String,
}

impl Display for RosAction {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        let h = self.header();
        let d = self.params_doc();
        let def = &self.show
            .replace("|header|", &h)
            .replace("|params_doc|", &d);
        write!(f, "{}", def)
    }
}

impl RosAction {
    pub fn new(name: String, mut params: Vec<RosParam>, show: String) -> Self {
        params.push(RosParam::Input(Param::new("name", MesType::String)));
        Self { name, params, show }
    }

    pub fn header(&self) -> String {
        let params = &self.params.iter().map(|p| p.param()).join(", ");
        let name = &self.name;
        format!("{name}({params})")
    }
    pub fn params_doc(&self) -> String {
        let params = &self.params.iter().map(|p| {
            match p {
                RosParam::Input(param) => {
                    let p = param.to_string();
                    format!("// - input parameter: {p}")
                }
                RosParam::InputWithDefault(param, default) => {
                    let p = param.to_string();
                    format!("// - input parameter: {p}, default value: {default}")
                }
                RosParam::Output(param) => {
                    let p = param.to_string();
                    format!("// - output parameter: {p}")
                }
            }
        }).join("\n");
        format!(r#"// Parameters:
{params}"#)
    }
}

#[derive(Debug, Clone)]
pub enum RosParam {
    Input(Param),
    InputWithDefault(Param, Message),
    Output(Param),
}

impl RosParam {
    fn param(&self) -> &Param {
        match self {
            RosParam::Input(p) => p,
            RosParam::InputWithDefault(p, _) => p,
            RosParam::Output(p) => p,
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::runtime::builder::ros_nav::{ros_actions_file};

    #[test]
    fn print() {
        println!("{}", ros_actions_file())
    }
}