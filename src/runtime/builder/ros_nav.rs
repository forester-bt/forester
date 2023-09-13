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
        "_" => Ok(Action::sync(ReturnResult::success())),

        _ => Err(RuntimeError::UnImplementedAction(format!(
            "action {action} is absent in the library"
        ))),
    }
}

pub fn ros_actions_file() -> String {
    let actions = ros_actions().iter().map(|(_, v)| v.to_string()).join("\n");
    format!(r#"
// Ros specific actions and decorators.
// The actions are accessible using the import 'import "ros::nav2"'

// Actions
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
                RosParam::Input(Param::new("server_name", MesType::String)),
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
                RosParam::Output(Param::new("smoothed_path", MesType::String) ),
                RosParam::Output(Param::new("smoothing_duration", MesType::Num) ),
                RosParam::Output(Param::new("was_completed", MesType::Bool) ),
                RosParam::Output(Param::new("error_code_id", MesType::Num) ),
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
    ])
}

fn kv(a: RosAction) -> (String, RosAction) {
    (a.name.clone(), a)
}

pub struct RosAction {
    name: String,
    params: Vec<RosParam>,
    show: String,
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