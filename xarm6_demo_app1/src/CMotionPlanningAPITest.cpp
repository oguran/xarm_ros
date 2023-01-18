#include <xarm6_demo_app1/CMotionPlanningAPITest.h>
#include <xarm6_demo_app1/Utility.h>

// MoveIt
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>


CMotionPlanningAPITest::CMotionPlanningAPITest(ros::NodeHandle& node_handle, const std::string model_frame, const std::string planning_group)
  : node_handle_(node_handle), robot_base_frame_(model_frame), visual_tools_(model_frame), planning_group_(planning_group), robot_model_loader_("robot_description")
{
}


bool CMotionPlanningAPITest::Test() {
  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using a planner is pretty easy. Planners are
  // setup as plugins in MoveIt and you can use the ROS pluginlib
  // interface to load any planner that you want to use. Before we
  // can load the planner, we need two objects, a RobotModel and a
  // PlanningScene. We will start by instantiating a `RobotModelLoader`_
  // object, which will look up the robot description on the ROS
  // parameter server and construct a :moveit_core:`RobotModel` for us
  // to use.
  //
  // .. _RobotModelLoader:
  //     http://docs.ros.org/indigo/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model::RobotModelPtr robot_model = robot_model_loader_.getModel();
  /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(planning_group_);

  // Using the :moveit_core:`RobotModel`, we can construct a :planning_scene:`PlanningScene`
  // that maintains the state of the world (including the robot).
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  // Configure a valid robot state
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  // We will get the name of planning plugin we want to load
  // from the ROS parameter server, and then load the planner
  // making sure to catch all exceptions.
  if (!node_handle_.getParam("/move_group/planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle_.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
        << "Available plugins: " << ss.str());
  }

  // Visualization
  // ^^^^^^^^^^^^^
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  visual_tools_.loadRobotStatePub("/display_robot_state");
  visual_tools_.enableBatchPublishing();
  visual_tools_.deleteAllMarkers();  // clear all old markers
  visual_tools_.trigger();

  /* Remote control is an introspection tool that allows users to step through a high level script
     via buttons and keyboard shortcuts in RViz */
  visual_tools_.loadRemoteControl();

  /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools_.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

  /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
  visual_tools_.trigger();


  /* We can also use visual_tools to wait for user input */
  visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Pose Goal
  // ^^^^^^^^^
  // We will now create a motion plan request for the arm of the xarm 
  // specifying the desired pose of the end-effector as input.
  visual_tools_.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools_.trigger();
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = robot_base_frame_;
  pose.pose.position.x = 0.354;
  pose.pose.position.y = 0.037;
  pose.pose.position.z = 0.505;
  pose.pose.orientation.x = 1.0;

  // A tolerance of 0.01 m is specified in position
  // and 0.01 radians in orientation
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // We will create the request as a constraint using a helper function available
  // from the
  // `kinematic_constraints`_
  // package.
  //
  // .. _kinematic_constraints:
  //     http://docs.ros.org/indigo/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
  moveit_msgs::Constraints pose_goal =
    kinematic_constraints::constructGoalConstraints("link_tcp", pose, tolerance_pose, tolerance_angle);

  req.group_name = planning_group_;
  req.goal_constraints.push_back(pose_goal);

  // We now construct a planning context that encapsulate the scene,
  // the request and the response. We call the planner using this
  // planning context
  planning_interface::PlanningContextPtr context =
    planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
  ros::Publisher display_publisher =
    node_handle_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* Visualize the trajectory */
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools_.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools_.trigger();
  display_publisher.publish(display_trajectory);

  /* Set the state in the planning scene to the final state of the last plan */
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // Display the goal state
  visual_tools_.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools_.publishAxisLabeled(pose.pose, "goal_1");
  visual_tools_.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
  visual_tools_.trigger();

  /* We can also use visual_tools to wait for user input */
  visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Joint Space Goals
  // ^^^^^^^^^^^^^^^^^
  // Now, setup a joint space goal
  robot_state::RobotState goal_state(robot_model);
  std::vector<double> joint_values = { -1.0, 0.7, -0.2, -1.5, -0.7, 2.0};
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // Call the planner and visualize the trajectory
  /* Re-construct the planning context */
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  /* Call the Planner */
  context->solve(res);
  /* Check that the planning was successful */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  /* Visualize the trajectory */
  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);

  /* Now you should see two planned trajectories in series*/
  visual_tools_.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools_.trigger();
  display_publisher.publish(display_trajectory);

  /* We will add more goals. But first, set the state in the planning
     scene to the final state of the last plan */
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // Display the goal state
  visual_tools_.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools_.publishAxisLabeled(pose.pose, "goal_2");
  visual_tools_.publishText(text_pose, "Joint Space Goal (2)", rvt::WHITE, rvt::XLARGE);
  visual_tools_.trigger();

  /* Wait for user input */
  visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  /* Now, we go back to the first goal to prepare for orientation constrained planning */
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  res.getMessage(response);

  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools_.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools_.trigger();
  display_publisher.publish(display_trajectory);

  /* Set the state in the planning scene to the final state of the last plan */
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // Display the goal state
  visual_tools_.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools_.trigger();

  /* Wait for user input */
  visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Adding Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // Let's add a new pose goal again. This time we will also add a path constraint to the motion.
  /* Let's create a new pose goal */

  pose.pose.position.x = 0.32;
  pose.pose.position.y = -0.25;
  pose.pose.position.z = 0.65;
  pose.pose.orientation.w = 1.0;
  moveit_msgs::Constraints pose_goal_2 =
    kinematic_constraints::constructGoalConstraints("link_tcp", pose, tolerance_pose, tolerance_angle);

  /* Now, let's try to move to this new pose goal*/
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal_2);

  /* But, let's impose a path constraint on the motion.
     Here, we are asking for the end-effector to stay level*/
  geometry_msgs::QuaternionStamped quaternion;
  quaternion.header.frame_id = robot_base_frame_;
  quaternion.quaternion.w = 1.0;
  req.path_constraints = kinematic_constraints::constructGoalConstraints("link_tcp", quaternion);

  // Imposing path constraints requires the planner to reason in the space of possible positions of the end-effector
  // (the workspace of the robot)
  // because of this, we need to specify a bound for the allowed planning volume as well;
  // Note: a default bound is automatically filled by the WorkspaceBounds request adapter (part of the OMPL pipeline,
  // but that is not being used in this example).
  // We use a bound that definitely includes the reachable space for the arm. This is fine because sampling is not done
  // in this volume
  // when planning for the arm; the bounds are only used to determine if the sampled configurations are valid.
  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
    req.workspace_parameters.min_corner.z = -2.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
    req.workspace_parameters.max_corner.z = 2.0;

  // Call the planner and visualize all the plans created so far.
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools_.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools_.trigger();
  display_publisher.publish(display_trajectory);

  /* Set the state in the planning scene to the final state of the last plan */
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // Display the goal state
  visual_tools_.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools_.publishAxisLabeled(pose.pose, "goal_3");
  visual_tools_.publishText(text_pose, "Orientation Constrained Motion Plan (3)", rvt::WHITE, rvt::XLARGE);
  visual_tools_.trigger();

  // END_TUTORIAL
  /* Wait for user input */
  visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to exit the demo");
  planner_instance.reset();

  return true;
}
