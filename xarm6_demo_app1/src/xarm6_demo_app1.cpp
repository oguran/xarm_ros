/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

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
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>
#include <depth_image_proc/depth_traits.h>

/**
 * @brief RPYからクオータニオンを取得する関数
 *
 * @param roll [rad]
 * @param pitch [rad]
 * @param yaw [rad]
 * @param[out] q クオータニオン
 */
void GetQuaternionMsg(
    double roll,double pitch,double yaw,
    geometry_msgs::Quaternion &q
    ){
  tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
  quaternionTFToMsg(quat,q);
}

/**
*  @brief ROSのトピックのクオータニオンの構造体から
*         Roll,Pitch,Yaw角を取得する関数
*  @param q トピックのクオータニオン
*  @param[out] roll [rad]
*  @param[out] pitch [rad]
*  @param[out] yaw [rad]
*/
void GetRPY(const geometry_msgs::Quaternion &q,
    double &roll,double &pitch,double &yaw){
  tf::Quaternion quat(q.x,q.y,q.z,q.w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

bool SwitchController(ros::NodeHandle& node_handle,
    std::vector<std::string> start_controller,
    std::vector<std::string> stop_controller) {

  ros::ServiceClient switch_controller =
    node_handle.serviceClient<controller_manager_msgs::SwitchController>("xarm/controller_manager/switch_controller");
  controller_manager_msgs::SwitchController switch_controller_req;
  switch_controller_req.request.start_controllers = start_controller;
  switch_controller_req.request.stop_controllers = stop_controller;
  switch_controller_req.request.strictness = switch_controller_req.request.STRICT;
  ros::service::waitForService("xarm/controller_manager/switch_controller", ros::Duration(5));
  switch_controller.call(switch_controller_req);
  if (switch_controller_req.response.ok)
  {
    ROS_INFO_STREAM("Controller switch correctly");
  }
  else
  {
    ROS_ERROR_STREAM("Error occured trying to switch controller");
    return false;
  }

  return true;
}

#if 0
void ShowControllerStatistics(ros::NodeHandle& node_handle,) {
  ros::ServiceClient controller_statistics =
    node_handle.serviceClient<controller_manager_msgs::ControllerStatistics>("xarm/controller_manager/list_controllers");
  controller_manager_msgs::ListControllers list_controllers;
  ros::service::waitForService("xarm/controller_manager/list_controllers", ros::Duration(5));
}
#endif

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::Image, sensor_msgs::CameraInfo> DepthSyncPolicy;

class VisualServoTest {
  public:
    explicit VisualServoTest(ros::NodeHandle& node_handle)
      : arm_("xarm6"),
      tflistener_(tfBuffer_),
      gripper_("/xarm/gripper_controller/gripper_cmd", "true") {
        // 座標系をロボットのベースに基づいた「FIXED_FRAME」座標系を使う。
        arm_.setPoseReferenceFrame(FIXED_FRAME);
        gripper_.waitForServer();

        pub_arm_cartesian_ = node_handle.advertise<geometry_msgs::PoseStamped>("/xarm/xarm6_cartesian_motion_controller/goal", 1);
        pub_marker_= node_handle.advertise<visualization_msgs::Marker>("marker", 1);
        static message_filters::Subscriber<geometry_msgs::PoseStamped> sub_target(node_handle, "/xarm/camera/target", 1);
        static message_filters::Subscriber<sensor_msgs::Image> sub_image(node_handle, "/xarm/camera/depth/image", 1);
        static message_filters::Subscriber<sensor_msgs::CameraInfo> sub_cinfo(node_handle, "/xarm/camera/depth/camera_info", 1);
        static message_filters::Synchronizer<DepthSyncPolicy> sync(DepthSyncPolicy(10), sub_target, sub_image, sub_cinfo);
        sync.registerCallback(&VisualServoTest::DepthTargetCallback, this);
        ROS_INFO("Subscribe prepared!");
      }

    bool MoveToCognitionPose(ros::NodeHandle& node_handle) {
      ROS_INFO("Moving to cognition pose");
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = FIXED_FRAME;
      //pose.pose.position.x = 0.206873 - 0.50;
      //pose.pose.position.y = 0.0;
      //pose.pose.position.z = 0.111828 + 0.25;
      pose.pose.position.x = -0.354;
      pose.pose.position.y = -0.037;
      pose.pose.position.z = 0.505;

#if 0
      GetQuaternionMsg( 0.0, 3.14, 3.14, pose.pose.orientation);
#else
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 1.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 0.0;
#endif

      arm_.setPoseTarget(pose);
      if (!arm_.move()) {
        ROS_WARN("Could not move to cognition pose");
        return false;
      }

      ROS_INFO("Initialized pose gripper");
      control_msgs::GripperCommandGoal goal;
      goal.command.position = 0.3;
      gripper_.sendGoal(goal);
      bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
      if (!finishedBeforeTimeout) {
        ROS_WARN("gripper_ open action did not complete");
        return false;
      }
      ROS_INFO("Initialize gripper pose");

      return true;
    }

    bool DoApproach(ros::NodeHandle& node_handle) {
      ROS_INFO("Opening gripper");
      control_msgs::GripperCommandGoal goal;
      goal.command.position = 0.0;
      gripper_.sendGoal(goal);
      bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
      if (!finishedBeforeTimeout) {
        ROS_WARN("gripper_open action did not complete");
        return false;
      }

      ROS_INFO("Approaching");
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = FIXED_FRAME;
      {
        std::lock_guard<std::mutex> lock(mtx_);
        pose.pose.position.x = target_pose_.position.x;
        pose.pose.position.y = target_pose_.position.y;
        pose.pose.position.z = target_pose_.position.z + 0.15;

        pose.pose.orientation.x = target_pose_.orientation.x;
        pose.pose.orientation.y = target_pose_.orientation.y;
        pose.pose.orientation.z = target_pose_.orientation.z;
        pose.pose.orientation.w = target_pose_.orientation.w;
      }

        ROS_WARN("position = %f, %f, %f  orientation = %f, %f, %f, %f",
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
            );

      std::cout << "Approached Pose = " << target_pose_ << std::endl;
      approaced_pose_ = pose.pose;
      arm_.setPoseTarget(pose);
      if (!arm_.move()) {
        ROS_WARN("Could not approaching");
        return false;
      }


      return true;
    }

    bool PreGrasp(ros::NodeHandle& node_handle) {
      std::vector<std::string> start_controller;
      start_controller.push_back("xarm6_cartesian_motion_controller");
      std::vector<std::string> stop_controller;
      stop_controller.push_back("xarm6_traj_controller");

      if (false == SwitchController(node_handle, start_controller, stop_controller)) {
        return false;
      }
      ros::Duration(2).sleep();

      ROS_INFO("Moving to grasp pose");
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = FIXED_FRAME;

      {
        std::lock_guard<std::mutex> lock(mtx_);

        pose.pose.position.x = target_pose_.position.x;
        pose.pose.position.y = target_pose_.position.y;
        pose.pose.position.z = target_pose_.position.z + 0.12;

        pose.pose.orientation.x = target_pose_.orientation.x;
        pose.pose.orientation.y = target_pose_.orientation.y;
        pose.pose.orientation.z = target_pose_.orientation.z;
        pose.pose.orientation.w = target_pose_.orientation.w;
      }
      std::cout << "Grasp Pose = " << target_pose_ << std::endl;
      pub_arm_cartesian_.publish(pose);

      ROS_INFO("Moved to picking pose");

      return true;
    }

    bool VisualServo(ros::NodeHandle& node_handle) {
#if 0
      std::vector<std::string> start_controller;
      start_controller.push_back("xarm6_cartesian_motion_controller_velocity");
      std::vector<std::string> stop_controller;
      stop_controller.push_back("xarm6_traj_controller_velocity");

      if (false == SwitchController(node_handle, start_controller, stop_controller)) {
        return false;
      }
      ros::Duration(2).sleep();
#endif

      ROS_INFO("Moving to picking pose");
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = FIXED_FRAME;

      ros::Rate rate(30);
      while (ros::ok()) {
        {
          std::lock_guard<std::mutex> lock(mtx_);

          if (pregrasped_) break;

          pose.pose.position.x = target_pose_.position.x;
          pose.pose.position.y = target_pose_.position.y;
          pose.pose.position.z = target_pose_.position.z;

          pose.pose.orientation.x = target_pose_.orientation.x;
          pose.pose.orientation.y = target_pose_.orientation.y;
          pose.pose.orientation.z = target_pose_.orientation.z;
          pose.pose.orientation.w = target_pose_.orientation.w;
        }
        pub_arm_cartesian_.publish(pose);

        rate.sleep();
      }
#if 1
      pregrasped_= true;
#endif

      ROS_INFO("Moved to picking pose");

      return true;
    }

    bool Grasp(ros::NodeHandle& node_handle) {

      ROS_INFO("Start Grasp");
      control_msgs::GripperCommandGoal goal;
      goal.command.position = 0.4;
      gripper_.sendGoal(goal);
      bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
      if (!finishedBeforeTimeout) {
        ROS_WARN("gripper_ open action did not complete");
        return false;
      }
      ROS_INFO("Grasped");

      return true;
    }

    bool PostGrasp(ros::NodeHandle& node_handle) {
      ROS_INFO("Moving to PostGrasped pose");
      geometry_msgs::PoseStamped pose;

      pose.header.frame_id = FIXED_FRAME;
      pose.pose = approaced_pose_;
      pose.pose.position.z = approaced_pose_.position.z + 0.12;

      std::cout << "PostGrasped Pose = " << approaced_pose_ << std::endl;
      pub_arm_cartesian_.publish(pose);

      ROS_INFO("Moved to PostGrasped pose");

      return true;
    }

void DepthTargetCallback(const geometry_msgs::PoseStampedConstPtr& msg_target,
        const sensor_msgs::ImageConstPtr& msg_image,
        const sensor_msgs::CameraInfoConstPtr& msg_cinfo) {

      geometry_msgs::PoseStamped target_pose;
      geometry_msgs::PoseStamped approaching_pose;
      //ros::Time now = ros::Time::now();

      cam_model_.fromCameraInfo(msg_cinfo);
      uint16_t u16_z = (uint16_t)msg_target->pose.position.z;

      cv::Point2d rs_point(msg_target->pose.position.x, msg_target->pose.position.y);
      cv::Point3d rs_ray = cam_model_.projectPixelTo3dRay(rs_point);

      float target_d = depth_image_proc::DepthTraits<uint16_t>::toMeters(u16_z);

      target_pose.header = msg_target->header;
      target_pose.pose.position.x = rs_ray.x * target_d;
      target_pose.pose.position.y = rs_ray.y * target_d;
      target_pose.pose.position.z = target_d;
      // TODO カメラのZ軸とGripperのZ軸が90度ずれている
      GetQuaternionMsg(0, 0, -M_PI/2, target_pose.pose.orientation);

      //std::cout << "target_pose.pose = " << target_pose.pose << std::endl;
      {
        visualization_msgs::Marker marker;
        marker.header.frame_id = msg_image->header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.pose.position.x = target_pose.pose.position.x;
        marker.pose.position.y = target_pose.pose.position.y;
        marker.pose.position.z = target_pose.pose.position.z;
        marker.pose.orientation.x = target_pose.pose.orientation.x;
        marker.pose.orientation.y = target_pose.pose.orientation.y;
        marker.pose.orientation.z = target_pose.pose.orientation.z;
        marker.pose.orientation.w = target_pose.pose.orientation.w;
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        pub_marker_.publish(marker);
      }

      if (!tfBuffer_.canTransform(FIXED_FRAME, cam_model_.tfFrame(), ros::Time(0), ros::Duration(1.0))) {
        ROS_WARN("Could not lookup transform from world to %s, in duration %f [sec]",
            cam_model_.tfFrame().c_str(), 1.0f);
        return;
      }

      try {
        tfBuffer_.transform(target_pose, approaching_pose, FIXED_FRAME, ros::Duration(1.0));
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return;
      }

      //std::cout << "approaching_pose.pose = " << approaching_pose.pose << std::endl;
      {
        std::lock_guard<std::mutex> lock(mtx_);

        target_pose_.position = approaching_pose.pose.position;
        target_pose_.orientation = approaching_pose.pose.orientation;

        //if (target_pose_.position.z) pregrasped_= true;
      }
    }

  private:
    moveit::planning_interface::MoveGroupInterface arm_;
    moveit::planning_interface::PlanningSceneInterface scene_;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
    ros::Publisher pub_arm_cartesian_;
    ros::Publisher pub_marker_;
    const std::string PLANNING_GROUP = "xarm6";
    const std::string FIXED_FRAME = "world";
    geometry_msgs::Pose target_pose_;
    geometry_msgs::Pose approaced_pose_;
    std::mutex mtx_;
    bool pregrasped_ = false;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tflistener_;
    image_geometry::PinholeCameraModel cam_model_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xarm6_demo_app1_node");
  ros::NodeHandle node_handle;
  // MoveIt!はアシンクロナスな計算をしないといけないので、このコードによりROSのアシンクロナスな機能を初期化する。
  ros::AsyncSpinner spinner(10);
  spinner.start();

  VisualServoTest pnp(node_handle);
  pnp.MoveToCognitionPose(node_handle);
  pnp.DoApproach(node_handle);
  pnp.PreGrasp(node_handle);
      ros::Duration(2).sleep();
  //pnp.VisualServo(node_handle);
  pnp.Grasp(node_handle);
      ros::Duration(2).sleep();
  pnp.PostGrasp(node_handle);

  //spinner.stop();
  // Wait until the node is shut down
  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}
