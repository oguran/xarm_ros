#include <xarm6_demo_app1/CObjListManager.h>
#include <xarm6_demo_app1/Utility.h>

CObjListManager::CObjListManager(ros::NodeHandle& node_handle)
      : tflistener_(tfBuffer_)
{
  sub_cinfo_ = node_handle.subscribe("/camera/depth/camera_info", 10, &CObjListManager::CameraInfoCallback, this);
  sub_obj_pose_list_ = node_handle.subscribe("/srecog/obj_pose_list", 10, &CObjListManager::ObjPoseListCallback, this);
  sub_obj_point_list_ = node_handle.subscribe("/srecog/obj_point_list", 10, &CObjListManager::ObjPointListCallback, this);
  ROS_INFO("Subscribe prepared!");

  pub_marker_target_ = node_handle.advertise<visualization_msgs::Marker>("marker_target", 1);
  pub_marker_target_pose_list_ = node_handle.advertise<visualization_msgs::Marker>("marker_target_pose_lsit_", 1);
}

void CObjListManager::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg_cinfo) {
  if (rcv_cinfo) return;
  cam_model_.fromCameraInfo(msg_cinfo);
  rcv_cinfo = true;
}

  static int print_cnt = 0;
void CObjListManager::ObjPoseListCallback(const srecog_msgs::ObjPoseList& obj_pose_list) {
  if (obj_pose_list.obj_poses.empty()) {
    // 把持対象物が認識されていない場合は何もしない
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mtx_pose_);

    target_obj_pose_camera_.header = obj_pose_list.header;
    target_obj_pose_camera_.header.stamp = ros::Time::now();
    target_obj_pose_camera_.pose = obj_pose_list.obj_poses[0].poses[0];
  }

#if 0 // for debug
  if ((print_cnt++ % 10) == 0) {
    double roll, pitch, yaw;
    GetRPY(target_obj_pose_camera_.pose.orientation, roll, pitch, yaw);
    printEuler("srecog_pose_camera", roll, pitch, yaw);

    geometry_msgs::PoseStamped target_obj_pose_local;
    const std::string TARGET_OBJ_FRAME = "posecnn/00_potted_meat_can_01";
    try {
      tfBuffer_.transform(target_obj_pose_camera_, target_obj_pose_local, TARGET_OBJ_FRAME, ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      return;
    }

    GetRPY(target_obj_pose_local.pose.orientation, roll, pitch, yaw);
    printEuler("srecog_pose_local ", roll, pitch, yaw);
    ROS_INFO(" ----- ");
  }
#endif
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_obj_pose_camera_.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;

    //marker.type = visualization_msgs::Marker::SPHERE;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    copyPose(target_obj_pose_camera_.pose, marker.pose);
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    pub_marker_target_pose_list_.publish(marker);
    //std::cout << "marker.pose = " << marker.pose << std::endl;
  }

}

void CObjListManager::ObjPointListCallback(const srecog_msgs::ObjPointList& obj_point_list) {

  if (!rcv_cinfo) {
    ROS_WARN("Not receive camera_info yet.");
    return;
  }

  if (obj_point_list.obj_points.empty()) {
    // 把持対象物が認識されていない場合は何もしない
    return;
  }

  geometry_msgs::PoseStamped target_pose;
  geometry_msgs::PoseStamped approaching_pose;
  //ros::Time now = ros::Time::now();
  target_pose.header = obj_point_list.header;
  target_pose.pose.position = obj_point_list.obj_points[0].center;

  // TODO カメラのyawとGripperのyawが90度ずれている
  GetQuaternionMsg(0, 0, -M_PI/2, target_pose.pose.orientation);

  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = target_pose.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    copyPose(target_pose.pose, marker.pose);
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    pub_marker_target_.publish(marker);
    //std::cout << "marker.pose = " << marker.pose << std::endl;
  }

  if (!tfBuffer_.canTransform(FIXED_FRAME, target_pose.header.frame_id, ros::Time(0), ros::Duration(1.0))) {
    ROS_WARN("Could not lookup transform from world to %s, in duration %f [sec]",
        target_pose.header.frame_id.c_str(), 1.0f);
    return;
  }

  try {
    tfBuffer_.transform(target_pose, approaching_pose, FIXED_FRAME, ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return;
  }

#if 0
  std::cout << "target_pose.pose = " << approaching_pose.pose << std::endl;
  approaching_pose.pose.orientation.x = 0.0f;
  approaching_pose.pose.orientation.y = 1.0f;
  approaching_pose.pose.orientation.z = 0.0f;
  approaching_pose.pose.orientation.w = 0.0f;
#endif
  {
    std::lock_guard<std::mutex> lock(mtx_point_);

    target_pose_.position = approaching_pose.pose.position;
    target_pose_.orientation = approaching_pose.pose.orientation;
  }

  // 把持対象物のTFを作成＆broadcastする
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped tfs;

  tfs.header.frame_id = FIXED_FRAME;
  tfs.child_frame_id = TARGET_FRAME;
  tfs.transform.translation.x = target_pose_.position.x;
  tfs.transform.translation.y = target_pose_.position.y;
  tfs.transform.translation.z = target_pose_.position.z;
  tfs.transform.rotation.x = target_pose_.orientation.x;
  tfs.transform.rotation.y = target_pose_.orientation.y;
  tfs.transform.rotation.z = target_pose_.orientation.z;
  tfs.transform.rotation.w = target_pose_.orientation.w;;

  br.sendTransform(tfs);
}

