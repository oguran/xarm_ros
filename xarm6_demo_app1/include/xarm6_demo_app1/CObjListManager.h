/* Copyright 2020 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/Duration.h>
#include <mutex>
#include <math.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <srecog_msgs/ObjPoseList.h>
#include <srecog_msgs/ObjPointList.h>

#include <visualization_msgs/Marker.h>
#include <depth_image_proc/depth_traits.h>

class CObjListManager {
  public:
    explicit CObjListManager(ros::NodeHandle& node_handle);

    void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg_cinfo);
    void ObjPoseListCallback(const srecog_msgs::ObjPoseList& obj_pose_list);
    void ObjPointListCallback(const srecog_msgs::ObjPointList& obj_point_list);

  private:
    srecog_msgs::ObjPoseList obj_pose_list_;
    geometry_msgs::Pose target_pose_;
    std::mutex mtx_point_;
    std::mutex mtx_pose_;
    tf2_ros::Buffer tfBuffer_;
    image_geometry::PinholeCameraModel cam_model_;
    ros::Publisher pub_marker_target_;
    bool rcv_cinfo = false;

    const std::string FIXED_FRAME = "world";
    const std::string TARGET_FRAME = "target";
};
