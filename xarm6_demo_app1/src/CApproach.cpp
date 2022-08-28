#include <xarm6_demo_app1/CApproach.h>
#include <xarm6_demo_app1/Utility.h>


CApproach::CApproach(ros::NodeHandle& node_handle, CObjListManager& olm)
    : arm_("xarm6"),
    gripper_("/xarm/gripper_controller/gripper_cmd", "true"),
    node_handle(node_handle),
    olm(olm) {
        // 座標系をロボットのベースに基づいた「FIXED_FRAME」座標系を使う。
        arm_.setPoseReferenceFrame(FIXED_FRAME);
        gripper_.waitForServer();

        pub_marker_target_1st_ = node_handle.advertise<visualization_msgs::Marker>("marker_target_1st", 1);
        pub_marker_target_2nd_ = node_handle.advertise<visualization_msgs::Marker>("marker_target_2nd", 1);
        pub_marker_target_3rd_ = node_handle.advertise<visualization_msgs::Marker>("marker_target_3rd", 1);
        pub_marker_target_rot_= node_handle.advertise<visualization_msgs::Marker>("marker_target_rot", 1);
        ROS_INFO("Subscribe prepared!");
    }

bool CApproach::MoveToCognitionPose() {
    ROS_INFO("Moving to cognition pose");
    cognition_pose_.header.frame_id = FIXED_FRAME;
    cognition_pose_.pose.position.x = -0.354;
    cognition_pose_.pose.position.y = -0.037;
    cognition_pose_.pose.position.z = 0.505;

    cognition_pose_.pose.orientation.x = 0.0;
    cognition_pose_.pose.orientation.y = 1.0;
    cognition_pose_.pose.orientation.z = 0.0;
    cognition_pose_.pose.orientation.w = 0.0;

    arm_.setPoseTarget(cognition_pose_);
    if (!arm_.move()) {
        ROS_WARN("Could not move to cognition pose");
        return false;
    }

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
  if (!olm.tfBuffer_.canTransform(FIXED_FRAME, target_obj_pose_camera.header.frame_id, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from %s to %s, in duration %f [sec]",
        target_obj_pose_camera.header.frame_id.c_str(),
        FIXED_FRAME.c_str(),
        10.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(target_obj_pose_camera, target_pose_1st_, FIXED_FRAME, ros::Duration(10.0));
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

bool CApproach::DoApproach() {
    ROS_INFO("Opening gripper");
    control_msgs::GripperCommandGoal goal;
    goal.command.position = 0.0;
    gripper_.sendGoal(goal);
    bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(3));
    if (!finishedBeforeTimeout) {
        ROS_WARN("gripper_open action did not complete");
        return false;
    }

#if 0
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

#if 1
    arm_.setPoseTarget(approached_link_eef_pose_);
    if (!arm_.move()) {
        ROS_WARN("Could not approaching");
        return false;
    }
#endif

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

bool CApproach::DoApproachRotationTest() {
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

#if 1 // for debug
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

  tf_bc_.sendTransform(tfs);
#endif

  geometry_msgs::PoseStamped ps_camera_on_target_cordinate, ps_camera_on_camera_frame, ps_camera_on_fixed_frame;
  transformTFStampedToPoseStamped(tfs, ps_camera_on_target_cordinate);

  // 把持対象物 -> カメラ座標系を基準とした座標系に変換
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
        target_obj_pose_camera.header.frame_id, FIXED_FRAME, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from %s to %s, in duration %f [sec]",
        TARGET_FRAME.c_str(),
        target_obj_pose_camera.header.frame_id.c_str(),
        10.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(ps_camera_on_camera_frame, ps_camera_on_fixed_frame, FIXED_FRAME, ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  arm_.setPoseTarget(ps_camera_on_fixed_frame);
  if (!arm_.move()) {
    ROS_WARN("Could not approaching");
    return false;
  }

  return true;
}

bool CApproach::DoApproachRotationTest_backup() {
  ROS_INFO("Rotation");
  geometry_msgs::PoseStamped l_grasp_pose[2];
  geometry_msgs::PoseStamped l_2nd_pose, g_2nd_pose;

#if 1 // for debug
  approached_link_eef_pose_.header.stamp = ros::Time::now();
#endif

  grasp_pose_[PREGRASP_POSE].header = approached_link_eef_pose_.header;
  copyPose(approached_link_eef_pose_.pose, grasp_pose_[PREGRASP_POSE].pose);
  //grasp_pose_[PREGRASP_POSE].pose.position.z += PREGRASP_DISTANCE;

  printPose("Global_based", grasp_pose_[PREGRASP_POSE].pose);

  double roll, pitch, yaw;
  GetRPY(grasp_pose_[PREGRASP_POSE].pose.orientation, roll, pitch, yaw);
  printEuler("Global_based", roll, pitch, yaw);

  {
    visualization_msgs::Marker marker;
    marker.header = grasp_pose_[PREGRASP_POSE].header;
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    copyPose(grasp_pose_[PREGRASP_POSE].pose, marker.pose);
    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0f;
    pub_marker_target_2nd_.publish(marker);
  }

#if 1
  {
    geometry_msgs::TransformStamped tfs;

    tfs.header = grasp_pose_[PREGRASP_POSE].header;
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "grasp_pose_[PREGRASP_POSE]";
    tfs.transform.translation.x = grasp_pose_[PREGRASP_POSE].pose.position.x;
    tfs.transform.translation.y = grasp_pose_[PREGRASP_POSE].pose.position.y;
    tfs.transform.translation.z = grasp_pose_[PREGRASP_POSE].pose.position.z;
    tfs.transform.rotation.x = grasp_pose_[PREGRASP_POSE].pose.orientation.x;
    tfs.transform.rotation.y = grasp_pose_[PREGRASP_POSE].pose.orientation.y;
    tfs.transform.rotation.z = grasp_pose_[PREGRASP_POSE].pose.orientation.z;
    tfs.transform.rotation.w = grasp_pose_[PREGRASP_POSE].pose.orientation.w;;

    tf_bc_.sendTransform(tfs);
  }
#endif


  // 把持対象物を基準とした座標系に変換
  if (!olm.tfBuffer_.canTransform(TARGET_FRAME, FIXED_FRAME, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from world to %s, in duration %f [sec]",
        TARGET_FRAME.c_str(), 10.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(grasp_pose_[PREGRASP_POSE], l_grasp_pose[PREGRASP_POSE], TARGET_FRAME, ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  printPose("Target_based", l_grasp_pose[PREGRASP_POSE].pose);
  GetRPY(l_grasp_pose[PREGRASP_POSE].pose.orientation, roll, pitch, yaw);
  printEuler("Target_based", roll, pitch, yaw);

  {
    visualization_msgs::Marker marker;
    marker.header = l_grasp_pose[PREGRASP_POSE].header;
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    copyPose(l_grasp_pose[PREGRASP_POSE].pose, marker.pose);
    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    pub_marker_target_3rd_.publish(marker);
  }

#if 1
  {
    geometry_msgs::TransformStamped tfs;

    tfs.header = l_grasp_pose[PREGRASP_POSE].header;
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "l_grasp_pose[PREGRASP_POSE]";
    tfs.transform.translation.x = l_grasp_pose[PREGRASP_POSE].pose.position.x;
    tfs.transform.translation.y = l_grasp_pose[PREGRASP_POSE].pose.position.y;
    tfs.transform.translation.z = l_grasp_pose[PREGRASP_POSE].pose.position.z;
    tfs.transform.rotation.x = l_grasp_pose[PREGRASP_POSE].pose.orientation.x;
    tfs.transform.rotation.y = l_grasp_pose[PREGRASP_POSE].pose.orientation.y;
    tfs.transform.rotation.z = l_grasp_pose[PREGRASP_POSE].pose.orientation.z;
    tfs.transform.rotation.w = l_grasp_pose[PREGRASP_POSE].pose.orientation.w;;

    tf_bc_.sendTransform(tfs);
  }
#endif

  l_grasp_pose[GRASP_POSE].header = l_grasp_pose[PREGRASP_POSE].header;
  copyPose(l_grasp_pose[PREGRASP_POSE].pose, l_grasp_pose[GRASP_POSE].pose);
  l_grasp_pose[GRASP_POSE].pose.position.z += 0.14;

  l_2nd_pose.header = l_grasp_pose[GRASP_POSE].header;
  copyPose(l_grasp_pose[GRASP_POSE].pose, l_2nd_pose.pose);

  double rot_roll, rot_pitch, rot_yaw;
  GetRPY(target_pose_1st_.pose.orientation, rot_roll, rot_pitch, rot_yaw);

#if 0
  // Z axis (Yaw) rotation
  double theta_y = -rot_yaw;
  ROS_INFO("theta_y = %f", theta_y);
  l_grasp_pose[PREGRASP_POSE].pose.position.x = l_grasp_pose[PREGRASP_POSE].pose.position.x * cos(theta_y) - l_grasp_pose[PREGRASP_POSE].pose.position.y * sin(theta_y);
  l_grasp_pose[GRASP_POSE].pose.position.x = l_grasp_pose[GRASP_POSE].pose.position.x * cos(theta_y) - l_grasp_pose[GRASP_POSE].pose.position.y * sin(theta_y);
  l_grasp_pose[PREGRASP_POSE].pose.position.y = l_grasp_pose[PREGRASP_POSE].pose.position.x * sin(theta_y) + l_grasp_pose[PREGRASP_POSE].pose.position.y * cos(theta_y);
  l_grasp_pose[GRASP_POSE].pose.position.y = l_grasp_pose[GRASP_POSE].pose.position.x * sin(theta_y) + l_grasp_pose[GRASP_POSE].pose.position.y * cos(theta_y);
  yaw += theta_y;
#endif

#if 0
  // Y axis (Pitch) rotation
  //double theta_p = -rot_pitch;
  double theta_p = M_PI/6;
  ROS_INFO("theta_p = %f", theta_p);
  l_grasp_pose[PREGRASP_POSE].pose.position.x = l_grasp_pose[PREGRASP_POSE].pose.position.x * sin(theta_p) + l_grasp_pose[PREGRASP_POSE].pose.position.z * sin(theta_p);
  l_grasp_pose[GRASP_POSE].pose.position.x = l_grasp_pose[GRASP_POSE].pose.position.x * sin(theta_p) + l_grasp_pose[GRASP_POSE].pose.position.z * sin(theta_p);
  l_grasp_pose[PREGRASP_POSE].pose.position.z = -l_grasp_pose[PREGRASP_POSE].pose.position.x * sin(theta_p) + l_grasp_pose[PREGRASP_POSE].pose.position.z * cos(theta_p);
  // TODO tentative
  l_grasp_pose[GRASP_POSE].pose.position.z = -l_grasp_pose[GRASP_POSE].pose.position.x * sin(theta_p) + l_grasp_pose[GRASP_POSE].pose.position.z * cos(theta_p) + 0.035;
  pitch += theta_p;
#endif

#if 1
  // X axis (Roll) rotation
  //double theta_r = -rot_roll;
  double theta_r = M_PI/6;
  ROS_INFO("theta_r = %f", theta_r);
  l_grasp_pose[PREGRASP_POSE].pose.position.y = l_grasp_pose[PREGRASP_POSE].pose.position.y * cos(theta_r) - l_grasp_pose[PREGRASP_POSE].pose.position.z * sin(theta_r);
  l_grasp_pose[GRASP_POSE].pose.position.y = l_grasp_pose[GRASP_POSE].pose.position.y * cos(theta_r) - l_grasp_pose[GRASP_POSE].pose.position.z * sin(theta_r);
  l_grasp_pose[PREGRASP_POSE].pose.position.z = l_grasp_pose[PREGRASP_POSE].pose.position.y * sin(theta_r) + l_grasp_pose[PREGRASP_POSE].pose.position.z * cos(theta_r);
  l_grasp_pose[GRASP_POSE].pose.position.z = l_grasp_pose[GRASP_POSE].pose.position.y * sin(theta_r) + l_grasp_pose[GRASP_POSE].pose.position.z * cos(theta_r);
  roll += theta_r;
#endif

  GetQuaternionMsg(roll, pitch, yaw, l_grasp_pose[PREGRASP_POSE].pose.orientation);
  GetQuaternionMsg(roll, pitch, yaw, l_grasp_pose[GRASP_POSE].pose.orientation);

  grasp_pose_[POSTGRASP_POSE].header.frame_id = target_pose_1st_.header.frame_id;
  copyPose(target_pose_1st_.pose, grasp_pose_[POSTGRASP_POSE].pose);

  // Global座標を基準とした座標系に変換
  if (!olm.tfBuffer_.canTransform(FIXED_FRAME, TARGET_FRAME, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from world to %s, in duration %f [sec]",
        TARGET_FRAME.c_str(), 1.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(l_grasp_pose[PREGRASP_POSE], grasp_pose_[PREGRASP_POSE], FIXED_FRAME, ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  printPose("grasp_pose_[PREGRASP_POSE]", grasp_pose_[PREGRASP_POSE].pose);

  GetRPY(grasp_pose_[PREGRASP_POSE].pose.orientation, roll, pitch, yaw);
  printEuler("grasp_pose_[PREGRASP_POSE]", roll, pitch, yaw);

  try {
    olm.tfBuffer_.transform(l_grasp_pose[GRASP_POSE], grasp_pose_[GRASP_POSE], FIXED_FRAME, ros::Duration(1.0));
    olm.tfBuffer_.transform(l_2nd_pose, g_2nd_pose, FIXED_FRAME, ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  printPose("grasp_pose_[GRASP_POSE]", grasp_pose_[GRASP_POSE].pose);

  GetRPY(grasp_pose_[GRASP_POSE].pose.orientation, roll, pitch, yaw);
  printEuler("grasp_pose_[GRASP_POSE]", roll, pitch, yaw);

  {
    visualization_msgs::Marker marker;
    marker.header = l_grasp_pose[PREGRASP_POSE].header;
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    copyPose(l_grasp_pose[PREGRASP_POSE].pose, marker.pose);
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    pub_marker_target_rot_.publish(marker);
  }

#if 1
  {
    geometry_msgs::TransformStamped tfs;

    tfs.header = l_grasp_pose[PREGRASP_POSE].header;
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "l_grasp_pose[PREGRASP_POSE]_Ratated";
    tfs.transform.translation.x = l_grasp_pose[PREGRASP_POSE].pose.position.x;
    tfs.transform.translation.y = l_grasp_pose[PREGRASP_POSE].pose.position.y;
    tfs.transform.translation.z = l_grasp_pose[PREGRASP_POSE].pose.position.z;
    tfs.transform.rotation.x = l_grasp_pose[PREGRASP_POSE].pose.orientation.x;
    tfs.transform.rotation.y = l_grasp_pose[PREGRASP_POSE].pose.orientation.y;
    tfs.transform.rotation.z = l_grasp_pose[PREGRASP_POSE].pose.orientation.z;
    tfs.transform.rotation.w = l_grasp_pose[PREGRASP_POSE].pose.orientation.w;;

    tf_bc_.sendTransform(tfs);
  }
#endif

  {
    visualization_msgs::Marker marker;
    marker.header = grasp_pose_[PREGRASP_POSE].header;
    marker.ns = "basic_shapes";
    marker.id = 1;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    copyPose(grasp_pose_[PREGRASP_POSE].pose, marker.pose);
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.7f;
    pub_marker_target_rot_.publish(marker);
  }

#if 1
  {
    geometry_msgs::TransformStamped tfs;

    tfs.header = grasp_pose_[PREGRASP_POSE].header;
    tfs.header.stamp = ros::Time::now();
    tfs.child_frame_id = "grasp_pose_[PREGRASP_POSE]_Rotated";
    tfs.transform.translation.x = grasp_pose_[PREGRASP_POSE].pose.position.x;
    tfs.transform.translation.y = grasp_pose_[PREGRASP_POSE].pose.position.y;
    tfs.transform.translation.z = grasp_pose_[PREGRASP_POSE].pose.position.z;
    tfs.transform.rotation.x = grasp_pose_[PREGRASP_POSE].pose.orientation.x;
    tfs.transform.rotation.y = grasp_pose_[PREGRASP_POSE].pose.orientation.y;
    tfs.transform.rotation.z = grasp_pose_[PREGRASP_POSE].pose.orientation.z;
    tfs.transform.rotation.w = grasp_pose_[PREGRASP_POSE].pose.orientation.w;;

    tf_bc_.sendTransform(tfs);
  }
#endif

#if 0
  arm_.setPoseTarget(grasp_pose_[PREGRASP_POSE]);
  if (!arm_.move()) {
    ROS_WARN("Could not rotation");
    return false;
  }

  ros::Duration(2).sleep();

  geometry_msgs::TransformStamped approached_ts;
  try { // link_tcpの現座標を取得
    approached_ts = olm.tfBuffer_.lookupTransform(FIXED_FRAME, "link_tcp", ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  transformTFStampedToPoseStamped(approached_ts, approached_link_tcp_pose_);
#endif

  return true;
}


bool CApproach::DoApproachRotation() {
  ROS_INFO("Rotation");
  geometry_msgs::PoseStamped l_grasp_pose[2];
  geometry_msgs::PoseStamped l_2nd_pose, g_2nd_pose;
  grasp_pose_[PREGRASP_POSE].header.frame_id = FIXED_FRAME;
  {
    std::lock_guard<std::mutex> lock(olm.mtx_point_);
    copyPose(olm.target_pose_, grasp_pose_[PREGRASP_POSE].pose);
  }
  grasp_pose_[PREGRASP_POSE].pose.position.z += PREGRASP_DISTANCE;

  printPose("Global_based", grasp_pose_[PREGRASP_POSE].pose);

  double roll, pitch, yaw;
  GetRPY(grasp_pose_[PREGRASP_POSE].pose.orientation, roll, pitch, yaw);
  printEuler("Global_based", roll, pitch, yaw);

  {
    visualization_msgs::Marker marker;
    marker.header = grasp_pose_[PREGRASP_POSE].header;
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    copyPose(grasp_pose_[PREGRASP_POSE].pose, marker.pose);
    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 0.5f;
    marker.color.a = 1.0f;
    pub_marker_target_3rd_.publish(marker);
  }

  // 把持対象物を基準とした座標系に変換
  if (!olm.tfBuffer_.canTransform(TARGET_FRAME, FIXED_FRAME, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from world to %s, in duration %f [sec]",
        TARGET_FRAME.c_str(), 10.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(grasp_pose_[PREGRASP_POSE], l_grasp_pose[PREGRASP_POSE], TARGET_FRAME, ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  printPose("Target_based", l_grasp_pose[PREGRASP_POSE].pose);
  GetRPY(l_grasp_pose[PREGRASP_POSE].pose.orientation, roll, pitch, yaw);
  printEuler("Target_based", roll, pitch, yaw);

  l_grasp_pose[GRASP_POSE].header.frame_id = TARGET_FRAME;
  copyPose(l_grasp_pose[PREGRASP_POSE].pose, l_grasp_pose[GRASP_POSE].pose);
  l_grasp_pose[GRASP_POSE].pose.position.z += 0.14;

  l_2nd_pose.header.frame_id = TARGET_FRAME;
  copyPose(l_grasp_pose[GRASP_POSE].pose, l_2nd_pose.pose);

  double rot_roll, rot_pitch, rot_yaw;
  GetRPY(target_obj_pose_local_.pose.orientation, rot_roll, rot_pitch, rot_yaw);

  // Z axis (Yaw) rotation
  double theta_y = -rot_yaw;
  ROS_INFO("theta_y = %f", theta_y);
  l_grasp_pose[PREGRASP_POSE].pose.position.x = l_grasp_pose[PREGRASP_POSE].pose.position.x * cos(theta_y) - l_grasp_pose[PREGRASP_POSE].pose.position.y * sin(theta_y);
  l_grasp_pose[GRASP_POSE].pose.position.x = l_grasp_pose[GRASP_POSE].pose.position.x * cos(theta_y) - l_grasp_pose[GRASP_POSE].pose.position.y * sin(theta_y);
  l_grasp_pose[PREGRASP_POSE].pose.position.y = l_grasp_pose[PREGRASP_POSE].pose.position.x * sin(theta_y) + l_grasp_pose[PREGRASP_POSE].pose.position.y * cos(theta_y);
  l_grasp_pose[GRASP_POSE].pose.position.y = l_grasp_pose[GRASP_POSE].pose.position.x * sin(theta_y) + l_grasp_pose[GRASP_POSE].pose.position.y * cos(theta_y);
  yaw += theta_y;

  // Y axis (Pitch) rotation
  double theta_p = -rot_pitch;
  ROS_INFO("theta_p = %f", theta_p);
  l_grasp_pose[PREGRASP_POSE].pose.position.x = l_grasp_pose[PREGRASP_POSE].pose.position.x * sin(theta_p) + l_grasp_pose[PREGRASP_POSE].pose.position.z * sin(theta_p);
  l_grasp_pose[GRASP_POSE].pose.position.x = l_grasp_pose[GRASP_POSE].pose.position.x * sin(theta_p) + l_grasp_pose[GRASP_POSE].pose.position.z * sin(theta_p);
  l_grasp_pose[PREGRASP_POSE].pose.position.z = -l_grasp_pose[PREGRASP_POSE].pose.position.x * sin(theta_p) + l_grasp_pose[PREGRASP_POSE].pose.position.z * cos(theta_p);
  //l_grasp_pose[GRASP_POSE].pose.position.z = -l_grasp_pose[GRASP_POSE].pose.position.x * sin(theta_p) + l_grasp_pose[GRASP_POSE].pose.position.z * cos(theta_p);
  // TODO tentative
  l_grasp_pose[GRASP_POSE].pose.position.z = -l_grasp_pose[GRASP_POSE].pose.position.x * sin(theta_p) + l_grasp_pose[GRASP_POSE].pose.position.z * cos(theta_p) + 0.035;
  pitch += theta_p;

  // X axis (Roll) rotation
  double theta_r = -rot_roll;
  ROS_INFO("theta_r = %f", theta_r);
  l_grasp_pose[PREGRASP_POSE].pose.position.y = l_grasp_pose[PREGRASP_POSE].pose.position.y * cos(theta_r) - l_grasp_pose[PREGRASP_POSE].pose.position.z * sin(theta_r);
  l_grasp_pose[GRASP_POSE].pose.position.y = l_grasp_pose[GRASP_POSE].pose.position.y * cos(theta_r) - l_grasp_pose[GRASP_POSE].pose.position.z * sin(theta_r);
  l_grasp_pose[PREGRASP_POSE].pose.position.z = l_grasp_pose[PREGRASP_POSE].pose.position.y * sin(theta_r) + l_grasp_pose[PREGRASP_POSE].pose.position.z * cos(theta_r);
  l_grasp_pose[GRASP_POSE].pose.position.z = l_grasp_pose[GRASP_POSE].pose.position.y * sin(theta_r) + l_grasp_pose[GRASP_POSE].pose.position.z * cos(theta_r);
  roll += theta_r;

  GetQuaternionMsg(roll, pitch, yaw, l_grasp_pose[PREGRASP_POSE].pose.orientation);
  GetQuaternionMsg(roll, pitch, yaw, l_grasp_pose[GRASP_POSE].pose.orientation);

  grasp_pose_[POSTGRASP_POSE].header.frame_id = target_pose_1st_.header.frame_id;
  copyPose(target_pose_1st_.pose, grasp_pose_[POSTGRASP_POSE].pose);

  // Global座標を基準とした座標系に変換
  if (!olm.tfBuffer_.canTransform(FIXED_FRAME, TARGET_FRAME, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from world to %s, in duration %f [sec]",
        TARGET_FRAME.c_str(), 1.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(l_grasp_pose[PREGRASP_POSE], grasp_pose_[PREGRASP_POSE], FIXED_FRAME, ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  printPose("grasp_pose_[PREGRASP_POSE]", grasp_pose_[PREGRASP_POSE].pose);

  GetRPY(grasp_pose_[PREGRASP_POSE].pose.orientation, roll, pitch, yaw);
  printEuler("grasp_pose_[PREGRASP_POSE]", roll, pitch, yaw);

  try {
    olm.tfBuffer_.transform(l_grasp_pose[GRASP_POSE], grasp_pose_[GRASP_POSE], FIXED_FRAME, ros::Duration(1.0));
    olm.tfBuffer_.transform(l_2nd_pose, g_2nd_pose, FIXED_FRAME, ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  {
    visualization_msgs::Marker marker;
    marker.header = g_2nd_pose.header;
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    copyPose(g_2nd_pose.pose, marker.pose);
    marker.color.r = 0.5f;
    marker.color.g = 0.5f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    pub_marker_target_2nd_.publish(marker);
  }

  printPose("grasp_pose_[GRASP_POSE]", grasp_pose_[GRASP_POSE].pose);

  GetRPY(grasp_pose_[GRASP_POSE].pose.orientation, roll, pitch, yaw);
  printEuler("grasp_pose_[GRASP_POSE]", roll, pitch, yaw);

  {
    visualization_msgs::Marker marker;
    marker.header = grasp_pose_[GRASP_POSE].header;
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    copyPose(grasp_pose_[GRASP_POSE].pose, marker.pose);
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    pub_marker_target_rot_.publish(marker);
  }

#if 1
  arm_.setPoseTarget(grasp_pose_[PREGRASP_POSE]);
  if (!arm_.move()) {
    ROS_WARN("Could not rotation");
    return false;
  }

  ros::Duration(2).sleep();

  geometry_msgs::TransformStamped approached_ts;
  try { // link_tcpの現座標を取得
    approached_ts = olm.tfBuffer_.lookupTransform(FIXED_FRAME, "link_tcp", ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  transformTFStampedToPoseStamped(approached_ts, approached_link_tcp_pose_);
#endif

  return true;
}

