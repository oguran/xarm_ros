#include <xarm6_demo_app1/CApproach.h>
#include <xarm6_demo_app1/Utility.h>
#include <xarm6_demo_app1/CMovingAveragePose.h>

#define XARM_GRIPPER

CApproach::CApproach(ros::NodeHandle& node_handle, CObjListManager& olm, const std::string model_frame, std::string planning_group)
  : robot_base_frame_(model_frame),
  arm_(planning_group),
  gripper_("/xarm/gripper_controller/gripper_cmd", "true"),
  xarm_gripper_("xarm/gripper_move", "true"),
  visual_tools(model_frame),
  node_handle(node_handle),
  olm(olm) {
    // 座標系をロボットのベースに基づいた「robot_base_frame_」座標系を使う。
    arm_.setPoseReferenceFrame(robot_base_frame_);
#if defined(XARM_GRIPPER)
    xarm_gripper_.waitForServer();
#else
    gripper_.waitForServer();
#endif

    joint_model_group = arm_.getCurrentState()->getJointModelGroup(planning_group);

    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = -0.3;

    auto nhn = node_handle.getNamespace().c_str();
    ROS_INFO("Namespace is : %s", nhn);
    ROS_INFO("Planning frame: %s", arm_.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", arm_.getEndEffectorLink().c_str());

    pub_marker_target_1st_ = node_handle.advertise<visualization_msgs::Marker>("marker_target_1st", 1);
    pub_marker_target_2nd_ = node_handle.advertise<visualization_msgs::Marker>("marker_target_2nd", 1);
    pub_marker_target_3rd_ = node_handle.advertise<visualization_msgs::Marker>("marker_target_3rd", 1);
    pub_marker_target_rot_= node_handle.advertise<visualization_msgs::Marker>("marker_target_rot", 1);
    ROS_INFO("Subscribe prepared!");

  }

bool CApproach::MoveToHomePose(bool plan_confirm) {
  arm_.setNamedTarget(NAMED_POSE_HOME);
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("MoveToHomePose plan (pose goal) %s", success ? "" : "FAILED");

  if (plan_confirm) {
    ROS_INFO("MoveToHomePose plan as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(cognition_pose_.pose, "cognition_pose");
    visual_tools.publishText(text_pose, "Moving to home pose.", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to home pose.");
  }

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

bool CApproach::MoveToCognitionPose(bool plan_confirm) {
  ROS_INFO("Moving to cognition pose");
  cognition_pose_.header.frame_id = robot_base_frame_;
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

  if (plan_confirm) {
    ROS_INFO("MoveToCognitionPose plan as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(cognition_pose_.pose, "cognition_pose");
    visual_tools.publishText(text_pose, "Moving to cognition pose.", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to cognition pose.");
  }

  //if (!arm_.move()) {
  if (!arm_.execute(plan)) {
    ROS_WARN("Could not move to cognition pose");
    return false;
  }

  // cognition poseを記憶しておく
  arm_.rememberJointValues(COGNITION_POSE);

  if (plan_confirm) {
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window if cognition was finished.");
  }


  ROS_INFO("Initialized pose gripper");
#if defined(XARM_GRIPPER)
  xarm_gripper::MoveGoal goal;
  ROS_INFO("Use Initilized target_pulse: 450, pulse_speed: 1500");
  goal.target_pulse = 450;
  goal.pulse_speed = 1500;
  xarm_gripper_.sendGoal(goal);
  bool finishedBeforeTimeout = xarm_gripper_.waitForResult(ros::Duration(3));
  if (finishedBeforeTimeout) {
    ROS_WARN("xarm_gripper_ open action did not complete");
    xarm_gripper_.cancelAllGoals();
  }
#else
  control_msgs::GripperCommandActionGoal goal;
  goal.goal.command.position = 0.3;
  gripper_.sendGoal(goal.goal);
  bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(3));
  if (!finishedBeforeTimeout) {
    ROS_WARN("gripper_ open action did not complete");
    gripper_.cancelAllGoals();
  }
#endif
  ROS_INFO("Initialize gripper pose");

  return true;
}

bool CApproach::ObjPoseCognition() {
  geometry_msgs::PoseStamped target_obj_pose_camera;

  olm.enablePublishTargetTF();
  ros::Duration(1).sleep();

  // Moving Average Filter of Target Pose
  geometry_msgs::Pose pose;
  ros::Rate rate(AVERAGE_SAMPLING_RATE);
  int cnt = AVERAGE_SAMPLING_SIZE;
  CMovingAveragePose MAPose(cnt);
  while (ros::ok() && cnt--) {
    {
      std::lock_guard<std::mutex> lock(olm.mtx_point_);
      if (olm.vect_target_obj_pose_camera_.empty()) return false;
      target_obj_pose_camera.header = olm.vect_target_obj_pose_camera_.at(0).header;
      copyPose(olm.vect_target_obj_pose_camera_.at(0).pose, pose);
    }
    MAPose.averagedPose(pose, target_obj_pose_camera.pose);
    rate.sleep();
  }

  // カメラのframe_idを保持.
  camera_frame_ = target_obj_pose_camera.header.frame_id;

  // カメラ座標系 -> ロボット基準座標系に変換
  if (!olm.tfBuffer_.canTransform(robot_base_frame_, target_obj_pose_camera.header.frame_id, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from %s to %s, in duration %f [sec]",
        target_obj_pose_camera.header.frame_id.c_str(),
        robot_base_frame_.c_str(),
        10.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(target_obj_pose_camera, target_pose_, robot_base_frame_, ros::Duration(10.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  olm.disablePublishTargetTF();

  return true;
}

bool CApproach::DoApproach(bool plan_confirm) {
  ROS_INFO("Opening gripper");
#if defined(XARM_GRIPPER)
  xarm_gripper::MoveGoal goal;
  goal.target_pulse = 850;
  xarm_gripper_.sendGoal(goal);
  bool finishedBeforeTimeout = xarm_gripper_.waitForResult(ros::Duration(3));
  if (finishedBeforeTimeout) {
    ROS_WARN("xarm_gripper_ open action did not complete");
    xarm_gripper_.cancelAllGoals();
  }
#else
  control_msgs::GripperCommandGoal goal;
  goal.command.position = 0.0;
  gripper_.sendGoal(goal);
  bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(3));
  if (!finishedBeforeTimeout) {
    ROS_WARN("gripper_open action did not complete");
    gripper_.cancelAllGoals();
  }
#endif

  ROS_INFO("Approaching");
  copyPose(target_pose_.pose, approached_link_eef_pose_.pose);
  approached_link_eef_pose_.header = target_pose_.header;
  approached_link_eef_pose_.pose.position.x += PREGRASP_OFFSET_X;
  approached_link_eef_pose_.pose.position.z += PREGRASP_DISTANCE;
  approached_link_eef_pose_.pose.orientation = cognition_pose_.pose.orientation;

  printPose("link_eff", approached_link_eef_pose_.pose);
  double roll, pitch, yaw;
  GetRPY(approached_link_eef_pose_.pose.orientation, roll, pitch, yaw);
  printEuler("Approached", roll, pitch, yaw);

  arm_.setPoseTarget(approached_link_eef_pose_);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("MoveToHomePose plan (pose goal) %s", success ? "" : "FAILED");

  if (plan_confirm) {
    ROS_INFO("MoveToHomePose plan as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(approached_link_eef_pose_.pose, "approached_link_eef_pose_");
    visual_tools.publishText(text_pose, "Moving to approached pose.", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to approached pose.");
  }

  if (!arm_.execute(plan)) {
    ROS_WARN("Could not approaching");
    return false;
  }
  visual_tools.deleteAllMarkers();

  {
    visualization_msgs::Marker marker;
    marker.header = target_pose_.header;
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    copyPose(target_pose_.pose, marker.pose);
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    pub_marker_target_1st_.publish(marker);
  }

  return true;
}

bool CApproach::DoApproachRotation(bool plan_confirm) {
  ROS_INFO("Rotation");
  geometry_msgs::PoseStamped target_obj_pose_camera;

  olm.enablePublishTargetTF();
  ros::Duration(1).sleep();

  // Moving Average Filter of Target Pose
  geometry_msgs::Pose pose;
  ros::Rate rate(AVERAGE_SAMPLING_RATE);
  int cnt = AVERAGE_SAMPLING_SIZE;
  CMovingAveragePose MAPose(cnt);
  while (ros::ok() && cnt--) {
    {
      std::lock_guard<std::mutex> lock(olm.mtx_point_);
      if (olm.vect_target_obj_pose_camera_.empty()) return false;
      target_obj_pose_camera.header = olm.vect_target_obj_pose_camera_.at(1).header;
      copyPose(olm.vect_target_obj_pose_camera_.at(1).pose, pose);
    }
    MAPose.averagedPose(pose, target_obj_pose_camera.pose);
    rate.sleep();
  }

  olm.disablePublishTargetTF();

  // カメラ座標系 -> ロボット基準座標系に変換
  if (!olm.tfBuffer_.canTransform(robot_base_frame_, target_obj_pose_camera.header.frame_id, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from %s to %s, in duration %f [sec]",
        target_obj_pose_camera.header.frame_id.c_str(),
        robot_base_frame_.c_str(),
        10.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(target_obj_pose_camera, target_pose_, robot_base_frame_, ros::Duration(10.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  // targetを基準座標としたカメラフレームの座標を取得する
  geometry_msgs::TransformStamped tf_camera_on_target_cordinate;
  try{
    tf_camera_on_target_cordinate = olm.tfBuffer_.lookupTransform(
        TARGET_FRAME,
        target_obj_pose_camera.header.frame_id,
        ros::Time(0), ros::Duration(1.0));
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }

  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  GetRPY(target_obj_pose_camera.pose.orientation, roll, pitch, yaw);

  // targetを基準座標として、pose_recogから受け取ったターゲットの姿勢から回転行列をつくり
  // カメラフレームのposeに掛けて、target座標を中心に回転させる
  auto v_camera_on_target_cordinate = xarm6_demo_app1::Vector3(
      tf_camera_on_target_cordinate.transform.translation.x,
      tf_camera_on_target_cordinate.transform.translation.y,
      tf_camera_on_target_cordinate.transform.translation.z);
  EulerAngle rot_eulerangle(roll, pitch, yaw, EulerOrder::ZYX);

  RotationMatrix rotateMatrix = calculateRotationMatrix(rot_eulerangle);
  auto v_rotated = rotateMatrix * v_camera_on_target_cordinate;

  xarm6_demo_app1::Quaternion q_org  = xarm6_demo_app1::Quaternion(
      tf_camera_on_target_cordinate.transform.rotation.x,
      tf_camera_on_target_cordinate.transform.rotation.y,
      tf_camera_on_target_cordinate.transform.rotation.z,
      tf_camera_on_target_cordinate.transform.rotation.w
  );
  xarm6_demo_app1::Quaternion q_rotate  = calculateQuaternion(rot_eulerangle);
  xarm6_demo_app1::Quaternion q_rotated = q_rotate * q_org;

  geometry_msgs::TransformStamped tfs;

  tfs.header = tf_camera_on_target_cordinate.header;
  tfs.child_frame_id = tf_camera_on_target_cordinate.child_frame_id + "_Rotated";
  tfs.transform.translation.x = v_rotated.x;
  tfs.transform.translation.y = v_rotated.y;
  tfs.transform.translation.z = v_rotated.z;
  tfs.transform.rotation.x = q_rotated.x;
  tfs.transform.rotation.y = q_rotated.y;
  tfs.transform.rotation.z = q_rotated.z;
  tfs.transform.rotation.w = q_rotated.w;

  geometry_msgs::PoseStamped ps_camera_on_target_cordinate, ps_camera_on_fixed_frame;
  transformTFStampedToPoseStamped(tfs, ps_camera_on_target_cordinate);

  // 把持対象物座標系 -> ロボット座標系 に変換
  if (!olm.tfBuffer_.canTransform(
        TARGET_FRAME, robot_base_frame_, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from %s to %s, in duration %f [sec]",
        TARGET_FRAME.c_str(),
        robot_base_frame_.c_str(),
        10.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(ps_camera_on_target_cordinate, ps_camera_on_fixed_frame,
        robot_base_frame_, ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  // FIXME カメラフレームとLINK_TCPの姿勢のyawが90度ずれている補正
  xarm6_demo_app1::Quaternion q_camera  = xarm6_demo_app1::Quaternion(
      ps_camera_on_fixed_frame.pose.orientation.x,
      ps_camera_on_fixed_frame.pose.orientation.y,
      ps_camera_on_fixed_frame.pose.orientation.z,
      ps_camera_on_fixed_frame.pose.orientation.w
  );
  xarm6_demo_app1::Quaternion q_camera_rotated = q_camera * q_camera.rotationZ(-M_PI/2.0);
  ps_camera_on_fixed_frame.pose.orientation.x = q_camera_rotated.x;
  ps_camera_on_fixed_frame.pose.orientation.y = q_camera_rotated.y;
  ps_camera_on_fixed_frame.pose.orientation.z = q_camera_rotated.z;
  ps_camera_on_fixed_frame.pose.orientation.w = q_camera_rotated.w;

  // FIXME 上記の回転行列を掛けた際のQuaternionが示す姿勢の精度が足りてないと，kineticでは問題が起きるらしい．
  // （melodic以降は，API側で精度を担保してくれるらしい）
  // 本当は，quaternionの正規化を行うべきだが，正しい正規化処理方法が分かっていないので，とりあえずのやり方でしのぐ
  // https://github.com/tork-a/tork_moveit_tutorial/issues/45
  GetRPY(ps_camera_on_fixed_frame.pose.orientation, roll, pitch, yaw);
  GetQuaternionMsg(roll, pitch, yaw, ps_camera_on_fixed_frame.pose.orientation);

  arm_.setPoseTarget(ps_camera_on_fixed_frame);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("MoveToPreGrasp1 plan (pose goal) %s", success ? "" : "FAILED");

  if (plan_confirm) {
    ROS_INFO("MoveToPreGrasp1 plan as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(ps_camera_on_fixed_frame.pose, "pregrasp_1st_step_pose");
    visual_tools.publishText(text_pose, "Moving to 1st pregrasp pose.", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to pregrasp pose.");
  }

  if (!arm_.execute(plan)) {
    ROS_WARN("Could not change pose to pregrasp1");
    return false;
  }
  visual_tools.deleteAllMarkers();

  geometry_msgs::TransformStamped tf_link_tcp_on_fixed_frame, tf_camera_frame_on_fixed_frame;
  try { // ロボットフレームを基準としたlink_tcpの相対座標を取得
    tf_link_tcp_on_fixed_frame = olm.tfBuffer_.lookupTransform(
        robot_base_frame_, "link_tcp", ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  try { // ロボットフレームを基準としたlink_tcpの相対座標を取得
    tf_camera_frame_on_fixed_frame = olm.tfBuffer_.lookupTransform(
        robot_base_frame_, target_obj_pose_camera.header.frame_id, ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  // カメラフレームが狙ったゴール座標となるように、setPoseTargetに設定するlink_tcpの座標を算出する
  ps_camera_on_fixed_frame.pose.position.x +=
    tf_link_tcp_on_fixed_frame.transform.translation.x - tf_camera_frame_on_fixed_frame.transform.translation.x;
  ps_camera_on_fixed_frame.pose.position.y +=
    tf_link_tcp_on_fixed_frame.transform.translation.y - tf_camera_frame_on_fixed_frame.transform.translation.y;
  ps_camera_on_fixed_frame.pose.position.z +=
    tf_link_tcp_on_fixed_frame.transform.translation.z - tf_camera_frame_on_fixed_frame.transform.translation.z;

  arm_.setPoseTarget(ps_camera_on_fixed_frame);

  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  success = (arm_.plan(plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("MoveToPreGrasp2 plan (pose goal) %s", success ? "" : "FAILED");

  if (plan_confirm) {
    ROS_INFO("MoveToPreGrasp2 plan as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(ps_camera_on_fixed_frame.pose, "ps_camera_on_fixed_framepregrasp_2nd_step_pose");
    visual_tools.publishText(text_pose, "Moving to 2nd pregrasp pose.", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan2.trajectory_, joint_model_group);
  }

  if (!arm_.execute(plan2)) {
    ROS_WARN("Could not change pose to pregrasp2");
    return false;
  }

  visual_tools.deleteAllMarkers();
  return true;
}


// Cognition Poseのz座標を維持しつつ，Targetの真上に移動する
bool CApproach::DoApproach_2(bool plan_confirm) {
  ROS_INFO("Opening gripper");
  control_msgs::GripperCommandGoal goal;
  goal.command.position = 0.0;
  gripper_.sendGoal(goal);
  bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(3));
  if (!finishedBeforeTimeout) {
    ROS_WARN("gripper_open action did not complete");
    //return false;
  }

  ROS_INFO("Congnition Pose 2nd");
  geometry_msgs::TransformStamped tf_link_tcp_on_fixed_frame, tf_camera_frame_on_fixed_frame;
  try { // ロボットフレームを基準としたlink_tcpの相対座標を取得
    tf_link_tcp_on_fixed_frame = olm.tfBuffer_.lookupTransform(robot_base_frame_, "link_tcp", ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  try { // ロボットフレームを基準としたcamera_frameの相対座標を取得
    tf_camera_frame_on_fixed_frame = olm.tfBuffer_.lookupTransform(robot_base_frame_, camera_frame_, ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  cognition_pose_2nd.header = target_pose_.header;
  cognition_pose_2nd.pose.position.x = target_pose_.pose.position.x
    + (tf_link_tcp_on_fixed_frame.transform.translation.x - tf_camera_frame_on_fixed_frame.transform.translation.x);
  cognition_pose_2nd.pose.position.y = target_pose_.pose.position.y
    + (tf_link_tcp_on_fixed_frame.transform.translation.y - tf_camera_frame_on_fixed_frame.transform.translation.y);
  cognition_pose_2nd.pose.position.z = cognition_pose_.pose.position.z;
  cognition_pose_2nd.pose.orientation = cognition_pose_.pose.orientation;

  copyPose(target_pose_.pose, approached_link_eef_pose_.pose);
  approached_link_eef_pose_.header = target_pose_.header;
  approached_link_eef_pose_.pose.position.z += PREGRASP_DISTANCE;
  approached_link_eef_pose_.pose.orientation = cognition_pose_.pose.orientation;

  printPose("approached_link_eef_pose_", approached_link_eef_pose_.pose);
  double roll, pitch, yaw;
  GetRPY(approached_link_eef_pose_.pose.orientation, roll, pitch, yaw);
  printEuler("approached_link_eef_pose_", roll, pitch, yaw);

  arm_.setPoseTarget(cognition_pose_2nd);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("MoveToHomePose plan (pose goal) %s", success ? "" : "FAILED");

  if (plan_confirm) {
    ROS_INFO("MoveToCongnitionPose2nd plan as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(cognition_pose_2nd.pose, "cognition_pose_2nd");
    visual_tools.publishText(text_pose, "Moving to CongnitionPose2nd.", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to approached pose.");
  }

  if (!arm_.execute(plan)) {
    ROS_WARN("Could not move");
    return false;
  }
  visual_tools.deleteAllMarkers();

  return true;
}

// TargetからPregrasp Distance上方の座標からTargetの推定姿勢分回転した移動をする
bool CApproach::DoApproachRotation_2(bool plan_confirm) {
  ROS_INFO("Rotation");
  geometry_msgs::PoseStamped target_obj_pose_on_camera, camera_pose_on_camera, camera_pose_on_target;

  olm.enablePublishTargetTF();
  ros::Duration(1).sleep();

  // Moving Average Filter of Target Pose
  geometry_msgs::Pose pose;
  ros::Rate rate(AVERAGE_SAMPLING_RATE);
  int cnt = AVERAGE_SAMPLING_SIZE;
  CMovingAveragePose MAPose(cnt);
  while (ros::ok() && cnt--) {
    {
      std::lock_guard<std::mutex> lock(olm.mtx_point_);
      if (olm.vect_target_obj_pose_camera_.empty()) return false;
      target_obj_pose_on_camera.header = olm.vect_target_obj_pose_camera_.at(0).header;
      copyPose(olm.vect_target_obj_pose_camera_.at(0).pose, pose);
    }
    MAPose.averagedPose(pose, target_obj_pose_on_camera.pose);
    rate.sleep();
  }

  olm.disablePublishTargetTF();

  // カメラ座標系でのカメラフレームのposeは {0, 0, 0, 0, 0, 0, 1.0}
  // 後の処理でTarget中心にPREGRASP_DISTANCEを半径としてカメラをRotationさせるために，z座標からPREGRASP_DISTANCEを引く
  camera_pose_on_camera = target_obj_pose_on_camera;
  camera_pose_on_camera.pose.position.x = 0.0;
  camera_pose_on_camera.pose.position.y = 0.0;
  camera_pose_on_camera.pose.position.z = target_obj_pose_on_camera.pose.position.z - PREGRASP_DISTANCE;
  camera_pose_on_camera.pose.orientation.x = 0.0;
  camera_pose_on_camera.pose.orientation.y = 0.0;
  camera_pose_on_camera.pose.orientation.z = 0.0;
  camera_pose_on_camera.pose.orientation.w = 1.0;

  // カメラ座標系 -> Target基準座標系に変換
  if (!olm.tfBuffer_.canTransform(TARGET_FRAME, camera_pose_on_camera.header.frame_id, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from %s to %s, in duration %f [sec]",
        camera_pose_on_camera.header.frame_id.c_str(),
        robot_base_frame_.c_str(),
        10.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(camera_pose_on_camera, camera_pose_on_target, TARGET_FRAME, ros::Duration(10.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  printPose("target_obj_pose_on_camera", target_obj_pose_on_camera.pose);
  printPose("camera_pose_on_target", camera_pose_on_target.pose);

  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  GetRPY(target_obj_pose_on_camera.pose.orientation, roll, pitch, yaw);

  // targetを基準座標として、pose_recogから受け取ったターゲットの姿勢から回転行列をつくり
  // カメラフレームのposeに掛けて、target座標を中心に回転させる
  // ただし，回転させるカメラフレームのposeのｚ座標はPREGRASP_DISTANCEとする
  auto v_camera_on_target_cordinate = xarm6_demo_app1::Vector3(
      camera_pose_on_target.pose.position.x,
      camera_pose_on_target.pose.position.y,
      camera_pose_on_target.pose.position.z);
  EulerAngle rot_eulerangle(roll, pitch, yaw, EulerOrder::ZYX);

  RotationMatrix rotateMatrix = calculateRotationMatrix(rot_eulerangle);
  auto v_rotated = rotateMatrix * v_camera_on_target_cordinate;

  xarm6_demo_app1::Quaternion q_org  = xarm6_demo_app1::Quaternion(
      camera_pose_on_target.pose.orientation.x,
      camera_pose_on_target.pose.orientation.y,
      camera_pose_on_target.pose.orientation.z,
      camera_pose_on_target.pose.orientation.w
  );
  xarm6_demo_app1::Quaternion q_rotate  = calculateQuaternion(rot_eulerangle);
  xarm6_demo_app1::Quaternion q_rotated = q_rotate * q_org;

  geometry_msgs::TransformStamped tfs;

  // targetを基準座標としたカメラフレームの座標を取得する
  geometry_msgs::TransformStamped tf_camera_on_target_cordinate;
  try{
    tf_camera_on_target_cordinate = olm.tfBuffer_.lookupTransform(
        TARGET_FRAME,
        target_obj_pose_on_camera.header.frame_id,
        ros::Time(0), ros::Duration(1.0));
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }

  tfs.header = tf_camera_on_target_cordinate.header;
  tfs.child_frame_id = tf_camera_on_target_cordinate.child_frame_id + "_Rotated";
  tfs.transform.translation.x = v_rotated.x;
  tfs.transform.translation.y = v_rotated.y;
  tfs.transform.translation.z = v_rotated.z;
  tfs.transform.rotation.x = q_rotated.x;
  tfs.transform.rotation.y = q_rotated.y;
  tfs.transform.rotation.z = q_rotated.z;
  tfs.transform.rotation.w = q_rotated.w;

  geometry_msgs::PoseStamped ps_camera_on_target_cordinate, ps_camera_on_fixed_frame;
  transformTFStampedToPoseStamped(tfs, ps_camera_on_target_cordinate);

  // 把持対象物座標系 -> ロボット座標系 に変換
  if (!olm.tfBuffer_.canTransform(
        TARGET_FRAME, robot_base_frame_, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from %s to %s, in duration %f [sec]",
        TARGET_FRAME.c_str(),
        robot_base_frame_.c_str(),
        10.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(ps_camera_on_target_cordinate, ps_camera_on_fixed_frame,
        robot_base_frame_, ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  // FIXME カメラフレームとLINK_TCPの姿勢のyawが90度ずれている補正
  xarm6_demo_app1::Quaternion q_camera  = xarm6_demo_app1::Quaternion(
      ps_camera_on_fixed_frame.pose.orientation.x,
      ps_camera_on_fixed_frame.pose.orientation.y,
      ps_camera_on_fixed_frame.pose.orientation.z,
      ps_camera_on_fixed_frame.pose.orientation.w
  );
  xarm6_demo_app1::Quaternion q_camera_rotated = q_camera * q_camera.rotationZ(-M_PI/2.0);
  ps_camera_on_fixed_frame.pose.orientation.x = q_camera_rotated.x;
  ps_camera_on_fixed_frame.pose.orientation.y = q_camera_rotated.y;
  ps_camera_on_fixed_frame.pose.orientation.z = q_camera_rotated.z;
  ps_camera_on_fixed_frame.pose.orientation.w = q_camera_rotated.w;

  // FIXME 上記の回転行列を掛けた際のQuaternionが示す姿勢の精度が足りてないと，kineticでは問題が起きるらしい．
  // （melodic以降は，API側で精度を担保してくれるらしい）
  // 本当は，quaternionの正規化を行うべきだが，正しい正規化処理方法が分かっていないので，とりあえずのやり方でしのぐ
  // https://github.com/tork-a/tork_moveit_tutorial/issues/45
  GetRPY(ps_camera_on_fixed_frame.pose.orientation, roll, pitch, yaw);
  GetQuaternionMsg(roll, pitch, yaw, ps_camera_on_fixed_frame.pose.orientation);

  arm_.setPoseTarget(ps_camera_on_fixed_frame);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("MoveToPreGrasp1 plan (pose goal) %s", success ? "" : "FAILED");

  if (plan_confirm) {
    ROS_INFO("MoveToPreGrasp1 plan as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(ps_camera_on_fixed_frame.pose, "pregrasp_1st_step_pose");
    visual_tools.publishText(text_pose, "Moving to 1st pregrasp pose.", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to pregrasp pose.");
  }

  if (!arm_.execute(plan)) {
    ROS_WARN("Could not change pose to pregrasp1");
    return false;
  }
  visual_tools.deleteAllMarkers();

  arm_.setPoseTarget(ps_camera_on_fixed_frame);

  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  success = (arm_.plan(plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("MoveToPreGrasp2 plan (pose goal) %s", success ? "" : "FAILED");

  if (plan_confirm) {
    ROS_INFO("MoveToPreGrasp2 plan as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(ps_camera_on_fixed_frame.pose, "ps_camera_on_fixed_framepregrasp_2nd_step_pose");
    visual_tools.publishText(text_pose, "Moving to 2nd pregrasp pose.", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan2.trajectory_, joint_model_group);
  }

  if (!arm_.execute(plan2)) {
    ROS_WARN("Could not change pose to pregrasp2");
    return false;
  }

  visual_tools.deleteAllMarkers();
  return true;
}

