#include <xarm6_demo_app1/CMoveGroupTest.h>
#include <xarm6_demo_app1/Utility.h>
#include <moveit/robot_state/conversions.h>

CMoveGroupTest::CMoveGroupTest(ros::NodeHandle& node_handle, CObjListManager& olm)
  : arm_(PLANNING_GROUP),
  visual_tools(FIXED_FRAME),
  node_handle(node_handle),
  olm(olm) {
  //ac_("/xarm/xarm6_traj_controller/follow_joint_trajectory", true) {
    // 座標系をロボットのベースに基づいた「FIXED_FRAME」座標系を使う。
    arm_.setPoseReferenceFrame(FIXED_FRAME);

    joint_model_group = arm_.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = -0.3;

    auto nhn = node_handle.getNamespace().c_str();
    ROS_INFO("Namespace is : %s", nhn);
    ROS_INFO("Planning frame: %s", arm_.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", arm_.getEndEffectorLink().c_str());

    ROS_INFO("Subscribe prepared!");

  }

bool CMoveGroupTest::MoveToHomePose(bool plan_confirm) {
  arm_.setNamedTarget(NAMED_POSE_HOME);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("MoveToHomePose plan (pose goal) %s", success ? "" : "FAILED");

  ROS_INFO("MoveToHomePose plan as trajectory line");
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(cognition_pose_.pose, "cognition_pose");
  visual_tools.publishText(text_pose, "Moving to home pose.", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to home pose.");

  if (!arm_.execute(plan)) {
    ROS_WARN("Could not move to home pose");
    return false;
  }

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "xArm Pick & Place Demo", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo.");
  visual_tools.deleteAllMarkers();

  return true;
}

bool CMoveGroupTest::MoveToCognitionPose(bool plan_confirm) {
  ROS_INFO("Moving to cognition pose");
  cognition_pose_.header.frame_id = FIXED_FRAME;
  cognition_pose_.pose.position.x = 0.354;
  cognition_pose_.pose.position.y = 0.037;
  cognition_pose_.pose.position.z = 0.505;

  cognition_pose_.pose.orientation.x = 1.0;
  cognition_pose_.pose.orientation.y = 0.0;
  cognition_pose_.pose.orientation.z = 0.0;
  cognition_pose_.pose.orientation.w = 0.0;

  arm_.setPoseTarget(cognition_pose_);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("MoveToCognitionPose plan (pose goal) %s", success ? "" : "FAILED");

  ROS_INFO("MoveToCognitionPose plan as trajectory line");
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(cognition_pose_.pose, "cognition_pose");
  visual_tools.publishText(text_pose, "Moving to cognition pose.", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to cognition pose.");

  if (!arm_.execute(plan)) {
    ROS_WARN("Could not move to cognition pose");
    return false;
  }

  // cognition poseを記憶しておく
  arm_.rememberJointValues(COGNITION_POSE);
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window if cognition was finished.");

  return true;
}

bool CMoveGroupTest::ConstraintTest(bool plan_confirm) {
  ROS_INFO("Moving to cognition pose");
  cognition_pose_.header.frame_id = FIXED_FRAME;
  cognition_pose_.pose.position.x = 0.354;
  cognition_pose_.pose.position.y = 0.037;
  cognition_pose_.pose.position.z = 0.505;

  cognition_pose_.pose.orientation.x = 1.0;
  cognition_pose_.pose.orientation.y = 0.0;
  cognition_pose_.pose.orientation.z = 0.0;
  cognition_pose_.pose.orientation.w = 0.0;

  arm_.setPoseTarget(cognition_pose_);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("MoveToCognitionPose plan (pose goal) %s", success ? "" : "FAILED");

  ROS_INFO("MoveToCognitionPose plan as trajectory line");
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(cognition_pose_.pose, "cognition_pose");
  visual_tools.publishText(text_pose, "Moving to cognition pose.", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to cognition pose.");

  if (!arm_.execute(plan)) {
    ROS_WARN("Could not move to cognition pose");
    return false;
  }

  // cognition poseを記憶しておく
  arm_.rememberJointValues(COGNITION_POSE);

  // Test link1の制約条件を設定する
  geometry_msgs::TransformStamped tfs_link1;
  try { // link_tcpの現座標を取得
    tfs_link1 = olm.tfBuffer_.lookupTransform(FIXED_FRAME, "link1", ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "link1";
  ocm.header.frame_id = FIXED_FRAME;
  ocm.orientation.x = tfs_link1.transform.rotation.x;
  ocm.orientation.y = tfs_link1.transform.rotation.y;
  ocm.orientation.z = tfs_link1.transform.rotation.z;
  ocm.orientation.w = tfs_link1.transform.rotation.w;
  ocm.absolute_x_axis_tolerance = 3.14;
  ocm.absolute_y_axis_tolerance = 3.14;
  ocm.absolute_z_axis_tolerance = 3.14;
  ocm.weight = 0.5;

  moveit_msgs::Constraints constraints;
  constraints.orientation_constraints.push_back(ocm);
  arm_.setPathConstraints(constraints);

  // for joint space planning test
  moveit::core::RobotStatePtr current_state = arm_.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = -1.0;  // radians
  arm_.setJointValueTarget(joint_group_positions);

  success = (arm_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Test plan (joint space goal) %s", success ? "" : "FAILED");

  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  visual_tools.deleteAllMarkers();
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window if cognition was finished.");

  return true;
}


bool CMoveGroupTest::CartesianPathsTest(bool plan_confirm) {
  ROS_INFO("Execute CartesianPathsTest");

#if 0
  // wait for action server to come up
  while(!ac_.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the joint_trajectory_action server");
  }
#endif

  // Set waypoints as cartesian paths.
  geometry_msgs::Pose target_pose = arm_.getCurrentPose().pose;

  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.push_back(target_pose);

  target_pose.position.z -= 0.2;
  waypoints.push_back(target_pose);

  target_pose.position.y -= 0.2;
  waypoints.push_back(target_pose);


  target_pose.position.z += 0.2;
  target_pose.position.y += 0.2;
  target_pose.position.x -= 0.2;
  waypoints.push_back(target_pose);

  // Set maxiumum speed.
  arm_.setMaxVelocityScalingFactor(0.1);

  // Interpolated cattesian path at a resolution which set.
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshhold = 0.0;
  const double eef_step = 0.01;
  double fraction = arm_.computeCartesianPath(waypoints, eef_step, jump_threshhold, trajectory);
  ROS_INFO("Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);

  // Visualize the plan in Rviz.
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishPath(waypoints, rvt::LIME_GREEN, rvt::SMALL);
  for (std::size_t i = 0; i < waypoints.size(); ++i)
    visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rvt::SMALL);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

#if 0 // Unexecute moving, do not know why? It is bug?
#if 1
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory;
  moveit::core::robotStateToRobotStateMsg(*arm_.getCurrentState(), plan.start_state_);

  visual_tools.deleteAllMarkers();
  visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to home pose.");

  std::cout << "State : " << plan.start_state_ << std::endl;
  if (!arm_.execute(plan)) {
    ROS_WARN("Could not move to target pose");
    return false;
  }
#else
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory.joint_trajectory;
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  ac_.sendGoal(goal);
  std::cout << "Result = " << ac_.waitForResult() << std::endl;
  ROS_INFO("Move Goal Done.");
#endif
#endif

  return true;
}


bool CMoveGroupTest::AddRemoveAttachDetachObject(bool plan_confirm) {
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  //collision_object.header.frame_id = arm_.getPlanningFrame();
  collision_object.header.frame_id = FIXED_FRAME;

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.02;
  primitive.dimensions[2] = 0.1;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.position = arm_.getCurrentPose().pose.position;
  box_pose.position.y -= 0.15;
  box_pose.orientation.w = 1.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Wait for MoveGroup to recieve and process the collision object message
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object appears in RViz");

  // Now when we plan a trajectory it will avoid the obstacle
  arm_.setStartState(*arm_.getCurrentState());
  geometry_msgs::Pose another_pose;
  another_pose = arm_.getCurrentPose().pose;
  another_pose.position.y -= 0.25;
  arm_.setPoseTarget(another_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (arm_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // Now, let's attach the collision object to the robot.
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  arm_.attachObject(collision_object.id);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to recieve and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object attaches to the "
      "robot");

  // Now, let's detach the collision object from the robot.
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  arm_.detachObject(collision_object.id);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to recieve and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object detaches to the "
      "robot");

  // Now, let's remove the collision object from the world.
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in RViz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Wait for MoveGroup to recieve and process the attached collision object message */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to once the collision object disapears");
  return true;
}
