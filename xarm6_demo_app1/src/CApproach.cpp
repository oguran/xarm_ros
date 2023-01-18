#include <xarm6_demo_app1/CApproach.h>
#include <xarm6_demo_app1/Utility.h>

CApproach::CApproach(ros::NodeHandle& node_handle, CObjListManager& olm, const std::string model_frame, std::string planning_group)
  : robot_base_frame_(model_frame),
  arm_(planning_group),
  gripper_("/xarm/gripper_controller/gripper_cmd", "true"),
  visual_tools(model_frame),
  node_handle(node_handle),
  olm(olm) {
    // 座標系をロボットのベースに基づいた「robot_base_frame_」座標系を使う。
    arm_.setPoseReferenceFrame(robot_base_frame_);
    gripper_.waitForServer();

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

    ROS_INFO("MoveToCognitionPose plan as trajectory line");
    visual_tools.deleteAllMarkers();
    visual_tools.publishAxisLabeled(cognition_pose_.pose, "cognition_pose");
    visual_tools.publishText(text_pose, "Moving to cognition pose.", rvt::WHITE, rvt::XLARGE);
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to cognition pose.");

   //if (!arm_.move()) {
    if (!arm_.execute(plan)) {
        ROS_WARN("Could not move to cognition pose");
        return false;
    }

    // cognition poseを記憶しておく
    arm_.rememberJointValues(COGNITION_POSE);
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window if cognition was finished.");


    ROS_INFO("Initialized pose gripper");
    control_msgs::GripperCommandActionGoal goal;
    goal.goal.command.position = 0.3;
    gripper_.sendGoal(goal.goal);
    bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(3));
    if (!finishedBeforeTimeout) {
        ROS_WARN("gripper_ open action did not complete");
        return false;
    }
    ROS_INFO("Initialize gripper pose");

    return true;
}

bool CApproach::ObjPoseCognition() {
  geometry_msgs::PoseStamped target_obj_pose_camera;

  {
    std::lock_guard<std::mutex> lock(olm.mtx_pose_);
    if (olm.vect_target_obj_pose_camera_.empty()) return false;
    target_obj_pose_camera.header = olm.vect_target_obj_pose_camera_.at(0).header;
    copyPose(olm.vect_target_obj_pose_camera_.at(0).pose, target_obj_pose_camera.pose);
  }

  // カメラ座標系 -> ロボット基準座標系に変換
  if (!olm.tfBuffer_.canTransform(robot_base_frame_, target_obj_pose_camera.header.frame_id, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from %s to %s, in duration %f [sec]",
        target_obj_pose_camera.header.frame_id.c_str(),
        robot_base_frame_.c_str(),
        10.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(target_obj_pose_camera, target_pose_1st_, robot_base_frame_, ros::Duration(10.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

#if 1
// FIXME 本来はCObjListManagerで行いたいが、srecogが不正なデータを投げてくる場合があるので、暫定的にここで行う
  // 把持対象物のTFを作成＆broadcastする
  {
    geometry_msgs::TransformStamped tfs;

    tfs.header = target_pose_1st_.header;
    tfs.child_frame_id = TARGET_FRAME + "_based_on_robot";
    tfs.transform.translation.x = target_pose_1st_.pose.position.x;
    tfs.transform.translation.y = target_pose_1st_.pose.position.y;
    tfs.transform.translation.z = target_pose_1st_.pose.position.z;
    tfs.transform.rotation.x = target_pose_1st_.pose.orientation.x;
    tfs.transform.rotation.y = target_pose_1st_.pose.orientation.y;
    tfs.transform.rotation.z = target_pose_1st_.pose.orientation.z;
    tfs.transform.rotation.w = target_pose_1st_.pose.orientation.w;;

    tf_bc_.sendTransform(tfs);
  }

  // tfを投げてから反映されるまで少し待つ（いらないかも）
  ros::Duration(1).sleep();
#endif

  // カメラ座標系 -> 把持対象物を基準とした座標系に変換
  if (!olm.tfBuffer_.canTransform(TARGET_FRAME, target_obj_pose_camera.header.frame_id, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from %s to %s, in duration %f [sec]",
        target_obj_pose_camera.header.frame_id.c_str(),
        TARGET_FRAME.c_str(),
        10.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(target_obj_pose_camera, target_obj_pose_local_, TARGET_FRAME, ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  printPose("target_obj_pose_local_", target_obj_pose_local_.pose);
}

bool CApproach::DoApproach(bool plan_confirm) {
  ROS_INFO("Opening gripper");
  control_msgs::GripperCommandGoal goal;
  goal.command.position = 0.0;
  gripper_.sendGoal(goal);
  bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(3));
  if (!finishedBeforeTimeout) {
    ROS_WARN("gripper_open action did not complete");
    return false;
  }

#if 0 // Moving Average Filter of Target Pose
  geometry_msgs::Pose pose;
  ros::Rate rate(30);
  int cnt = 1;
  MovingAveragePose MAPose(cnt);
  while (ros::ok() && cnt--) {
    {
      std::lock_guard<std::mutex> lock(olm.mtx_point_);
      copyPose(olm.target_pose_, pose);
    }
    MAPose.averagedPose(pose, target_pose_1st_.pose);
    rate.sleep();
  }
#endif

  ROS_INFO("Approaching");
  copyPose(target_pose_1st_.pose, approached_link_eef_pose_.pose);
  approached_link_eef_pose_.header = target_pose_1st_.header;
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

  ROS_INFO("MoveToHomePose plan as trajectory line");
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(approached_link_eef_pose_.pose, "approached_link_eef_pose_");
  visual_tools.publishText(text_pose, "Moving to approached pose.", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to approached pose.");

  if (!arm_.execute(plan)) {
    ROS_WARN("Could not approaching");
    return false;
  }
  visual_tools.deleteAllMarkers();

  {
    visualization_msgs::Marker marker;
    marker.header = target_pose_1st_.header;
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    copyPose(target_pose_1st_.pose, marker.pose);
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

  {
    std::lock_guard<std::mutex> lock(olm.mtx_pose_);
    if (olm.vect_target_obj_pose_camera_.empty()) return false;
    target_obj_pose_camera.header = olm.vect_target_obj_pose_camera_.at(0).header;
    copyPose(olm.vect_target_obj_pose_camera_.at(0).pose, target_obj_pose_camera.pose);
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

#if 0 // for debug
  tf_camera_on_target_cordinate.child_frame_id = tf_camera_on_target_cordinate.child_frame_id + "_based_on_target";
  tf_bc_.sendTransform(tf_camera_on_target_cordinate);
#endif

  double roll = 0.0, pitch = 0.0, yaw = 0.0;
#if 1
  GetRPY(target_obj_pose_camera.pose.orientation, roll, pitch, yaw);
#else // for debug
  roll = 0.0;
  pitch = 0.0;
  yaw = M_PI/2.0;
#endif

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

#if 1 // for debug
  tf_bc_.sendTransform(tfs);
#endif

  geometry_msgs::PoseStamped ps_camera_on_target_cordinate, ps_camera_on_camera_frame, ps_camera_on_fixed_frame;
  transformTFStampedToPoseStamped(tfs, ps_camera_on_target_cordinate);

  // 把持対象物座標系 -> カメラ座標系 に変換
  if (!olm.tfBuffer_.canTransform(
        TARGET_FRAME, target_obj_pose_camera.header.frame_id, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from %s to %s, in duration %f [sec]",
        TARGET_FRAME.c_str(),
        target_obj_pose_camera.header.frame_id.c_str(),
        10.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(ps_camera_on_target_cordinate, ps_camera_on_camera_frame,
        target_obj_pose_camera.header.frame_id, ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  geometry_msgs::TransformStamped tf_link_tcp_on_camera_frame;
  try { // カメラフレームを基準としたlink_tcpの相対座標を取得
    tf_link_tcp_on_camera_frame = olm.tfBuffer_.lookupTransform(
        target_obj_pose_camera.header.frame_id, "link_tcp", ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

// TODO
#if 0
  // カメラフレームが狙ったゴール座標となるように、setPoseTargetに設定するlink_tcpの座標を算出する
  ps_camera_on_camera_frame.pose.position.x += tf_link_tcp_on_camera_frame.transform.translation.x;
  ps_camera_on_camera_frame.pose.position.y += tf_link_tcp_on_camera_frame.transform.translation.y;
  ps_camera_on_camera_frame.pose.position.z += tf_link_tcp_on_camera_frame.transform.translation.z;
  ps_camera_on_camera_frame.pose.orientation.x += tf_link_tcp_on_camera_frame.transform.rotation.x;
  ps_camera_on_camera_frame.pose.orientation.y += tf_link_tcp_on_camera_frame.transform.rotation.y;
  ps_camera_on_camera_frame.pose.orientation.z += tf_link_tcp_on_camera_frame.transform.rotation.z;
  ps_camera_on_camera_frame.pose.orientation.w += tf_link_tcp_on_camera_frame.transform.rotation.w;
#else
  GetRPY(ps_camera_on_camera_frame.pose.orientation, roll, pitch, yaw);
  GetQuaternionMsg(roll, pitch + M_PI/2.0, yaw, ps_camera_on_camera_frame.pose.orientation);
#endif


  // カメラ座標系 -> ロボットを基準とした座標系に変換
  if (!olm.tfBuffer_.canTransform(
        target_obj_pose_camera.header.frame_id, robot_base_frame_, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from %s to %s, in duration %f [sec]",
        TARGET_FRAME.c_str(),
        target_obj_pose_camera.header.frame_id.c_str(),
        10.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(ps_camera_on_camera_frame, ps_camera_on_fixed_frame, robot_base_frame_, ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  arm_.setPoseTarget(ps_camera_on_fixed_frame);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = (arm_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("MoveToHomePose plan (pose goal) %s", success ? "" : "FAILED");

  ROS_INFO("MoveToHomePose plan as trajectory line");
  visual_tools.publishAxisLabeled(ps_camera_on_fixed_frame.pose, "ps_camera_on_fixed_frame");
  visual_tools.publishText(text_pose, "Moving to pregrasp pose.", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to move to pregrasp pose.");

  if (!arm_.execute(plan)) {
    ROS_WARN("Could not change pose to pregrasp");
    return false;
  }

  return true;
}

