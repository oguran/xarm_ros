#include <xarm6_demo_app1/CObjListManager.h>
#include <xarm6_demo_app1/Utility.h>

CObjListManager::CObjListManager(ros::NodeHandle& node_handle)
      : tflistener_(tfBuffer_)
{
  //sub_cinfo_ = node_handle.subscribe("/camera/depth/camera_info", 10, &CObjListManager::CameraInfoCallback, this);
  sub_obj_pose_list_ = node_handle.subscribe("/srecog/obj_pose_list", 10, &CObjListManager::ObjPoseListCallback, this);
  sub_obj_point_list_ = node_handle.subscribe("/srecog/obj_point_list", 10, &CObjListManager::ObjPointListCallback, this);
  ROS_INFO("Subscribe prepared!");

  pub_marker_target_point_ = node_handle.advertise<visualization_msgs::Marker>("target_point", 1);
}

#if 0
void CObjListManager::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg_cinfo) {
  if (rcv_cinfo) return;
  cam_model_.fromCameraInfo(msg_cinfo);
  rcv_cinfo = true;
}
#endif

void CObjListManager::ObjPoseListCallback(const srecog_msgs::ObjPoseList& obj_pose_list) {
  if (obj_pose_list.obj_poses.empty()) {
    // 把持対象物が認識されていない場合は何もしない
    return;
  }

  geometry_msgs::PoseStamped target_obj_pose_camera;
  geometry_msgs::PoseStamped target_obj_pose_arm_base;

  target_obj_pose_camera.header = obj_pose_list.header;
  target_obj_pose_camera.pose = obj_pose_list.obj_poses[0].poses[0];

  {
    std::lock_guard<std::mutex> lock(mtx_pose_);

    vect_target_obj_pose_camera_.clear();
    std::for_each(std::begin(obj_pose_list.obj_poses[0].poses), std::end(obj_pose_list.obj_poses[0].poses),
        [&](geometry_msgs::Pose pose) {
        geometry_msgs::PoseStamped ps;
        ps.header = obj_pose_list.header;
        ps.pose = pose;
        vect_target_obj_pose_camera_.push_back(ps);
        });
  }

// FIXME srecogから不正なobj_pose_listが来た場合に、不正な把持対象物のTFを投げてしまう
// TODO Callback内 -> 別スレッドに処理移動
    // 把持対象物のTFを作成＆broadcastする
    geometry_msgs::TransformStamped tfs;

    tfs.header = target_obj_pose_camera.header;
    tfs.child_frame_id = TARGET_FRAME;
    tfs.transform.translation.x = target_obj_pose_camera.pose.position.x;
    tfs.transform.translation.y = target_obj_pose_camera.pose.position.y;
    tfs.transform.translation.z = target_obj_pose_camera.pose.position.z;
    tfs.transform.rotation.x = target_obj_pose_camera.pose.orientation.x;
    tfs.transform.rotation.y = target_obj_pose_camera.pose.orientation.y;
    tfs.transform.rotation.z = target_obj_pose_camera.pose.orientation.z;
    tfs.transform.rotation.w = target_obj_pose_camera.pose.orientation.w;

    tf_bc_.sendTransform(tfs);
}

void CObjListManager::ObjPointListCallback(const srecog_msgs::ObjPointList& obj_point_list) {

  if (obj_point_list.obj_points.empty()) {
    // 把持対象物が認識されていない場合は何もしない
    return;
  }

  {
    std::lock_guard<std::mutex> lock(mtx_point_);

    vect_target_obj_point_camera_.clear();
    std::for_each(std::begin(obj_point_list.obj_points), std::end(obj_point_list.obj_points),
        [&](srecog_msgs::ObjPoint point) {
        CObjPoint op(point.class_name, obj_point_list.header, point.center, point.left, point.right);
        vect_target_obj_point_camera_.push_back(op);
        });
  }

  { // for debug
    visualization_msgs::Marker marker;
    marker.header = obj_point_list.header;
    marker.ns = "basic_shapes";

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration();

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.id = 0;
    marker.pose.position.x = obj_point_list.obj_points[0].center.x;
    marker.pose.position.y = obj_point_list.obj_points[0].center.y;
    marker.pose.position.z = obj_point_list.obj_points[0].center.z;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    pub_marker_target_point_.publish(marker);

    marker.id = 1;
    marker.pose.position.x = obj_point_list.obj_points[0].left.x;
    marker.pose.position.y = obj_point_list.obj_points[0].left.y;
    marker.pose.position.z = obj_point_list.obj_points[0].left.z;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    pub_marker_target_point_.publish(marker);

    marker.id = 2;
    marker.pose.position.x = obj_point_list.obj_points[0].right.x;
    marker.pose.position.y = obj_point_list.obj_points[0].right.y;
    marker.pose.position.z = obj_point_list.obj_points[0].right.z;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    pub_marker_target_point_.publish(marker);
  }

}

#if 0
void ObjPointListCallback(const srecog_msgs::ObjPointList& obj_point_list) {
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
    pub_marker_target_center_.publish(marker);
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

}
#endif
