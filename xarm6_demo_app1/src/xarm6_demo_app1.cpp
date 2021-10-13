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

void transformTFStampedToPoseStamped(
    geometry_msgs::TransformStamped ts,
    geometry_msgs::PoseStamped& ps) {
  ps.header = ts.header;
  ps.pose.position.x = ts.transform.translation.x;
  ps.pose.position.y = ts.transform.translation.y;
  ps.pose.position.z = ts.transform.translation.z;
  ps.pose.orientation.x = ts.transform.rotation.x;
  ps.pose.orientation.y = ts.transform.rotation.y;
  ps.pose.orientation.z = ts.transform.rotation.z;
  ps.pose.orientation.w = ts.transform.rotation.w;
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
        pub_arm_cartesian_vel_ = node_handle.advertise<geometry_msgs::PoseStamped>("/xarm/xarm6_cartesian_motion_controller_velocity/goal", 1);
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
      //control_msgs::GripperCommandGoal goal;
      //goal.command.position = 0.3;
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

    bool DoApproach(ros::NodeHandle& node_handle) {
      ROS_INFO("Opening gripper");
      control_msgs::GripperCommandGoal goal;
      goal.command.position = 0.0;
      gripper_.sendGoal(goal);
      bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(3));
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
        pose.pose.position.z = target_pose_.position.z + 0.10;

        pose.pose.orientation.x = target_pose_.orientation.x;
        pose.pose.orientation.y = target_pose_.orientation.y;
        pose.pose.orientation.z = target_pose_.orientation.z;
        pose.pose.orientation.w = target_pose_.orientation.w;
      }

      ROS_INFO("link_eff position = %f, %f, %f orientation = %f, %f, %f, %f",
          pose.pose.position.x,
          pose.pose.position.y,
          pose.pose.position.z,
          pose.pose.orientation.x,
          pose.pose.orientation.y,
          pose.pose.orientation.z,
          pose.pose.orientation.w
          );
      double roll, pitch, yaw;
      GetRPY(pose.pose.orientation, roll, pitch, yaw);
      ROS_INFO("Approached roll, pitch, yaw = %f, %f, %f", (float)roll, (float)pitch, (float)yaw);


      arm_.setPoseTarget(pose);
      if (!arm_.move()) {
        ROS_WARN("Could not approaching");
        return false;
      }

      ros::Duration(2).sleep();

      geometry_msgs::TransformStamped approached_ts;
      try { // link_tcpの現座標を取得
        approached_ts = tfBuffer_.lookupTransform(FIXED_FRAME, "link_tcp", ros::Time(0), ros::Duration(1.0));
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
      }
      transformTFStampedToPoseStamped(approached_ts, approaced_pose_);

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

        pose.pose.position.x = target_pose_.position.x + 0.01;
        pose.pose.position.y = target_pose_.position.y;
        pose.pose.position.z = target_pose_.position.z - 0.07;

        pose.pose.orientation.x = target_pose_.orientation.x;
        pose.pose.orientation.y = target_pose_.orientation.y;
        pose.pose.orientation.z = target_pose_.orientation.z;
        pose.pose.orientation.w = target_pose_.orientation.w;
      }
      ROS_INFO("Grasp position = %f, %f, %f orientation = %f, %f, %f, %f",
          target_pose_.position.x,
          target_pose_.position.y,
          target_pose_.position.z,
          target_pose_.orientation.x,
          target_pose_.orientation.y,
          target_pose_.orientation.z,
          target_pose_.orientation.w
          );
      double roll, pitch, yaw;
      GetRPY(target_pose_.orientation, roll, pitch, yaw);
      ROS_INFO("Grasp roll, pitch, yaw = %f, %f, %f", (float)roll, (float)pitch, (float)yaw);

      pub_arm_cartesian_.publish(pose);

      ROS_INFO("Moved to picking pose");

      return true;
    }

    bool PreGraspVelocity(ros::NodeHandle& node_handle) {
      std::vector<std::string> start_controller;
      start_controller.push_back("xarm6_cartesian_motion_controller");
      std::vector<std::string> stop_controller;
      stop_controller.push_back("xarm6_traj_controller");

      if (false == SwitchController(node_handle, start_controller, stop_controller)) {
        return false;
      }
      ros::Duration(2).sleep();

      ROS_INFO("Moving to grasp pose");
      geometry_msgs::PoseStamped target_pose;
      target_pose.header.frame_id = FIXED_FRAME;

      {
        std::lock_guard<std::mutex> lock(mtx_);

        target_pose.pose.position.x = target_pose_.position.x + 0.01;
        target_pose.pose.position.y = target_pose_.position.y;
        //target_pose.pose.position.z = target_pose_.position.z - 0.07;
        target_pose.pose.position.z = target_pose_.position.z - 0.06;

        target_pose.pose.orientation.x = target_pose_.orientation.x;
        target_pose.pose.orientation.y = target_pose_.orientation.y;
        target_pose.pose.orientation.z = target_pose_.orientation.z;
        target_pose.pose.orientation.w = target_pose_.orientation.w;
      }

      ROS_INFO("Grasp Pose position = %f, %f, %f orientation = %f, %f, %f, %f",
          target_pose_.position.x,
          target_pose_.position.y,
          target_pose_.position.z,
          target_pose_.orientation.x,
          target_pose_.orientation.y,
          target_pose_.orientation.z,
          target_pose_.orientation.w
          );


      if (!CartesianVelCtrlOnPosCtrl(target_pose)) return false;

      ROS_INFO("Moved to picking pose");

      return true;
    }

    bool Grasp(ros::NodeHandle& node_handle) {

      ROS_INFO("Start Grasp");
      control_msgs::GripperCommandGoal goal;
      goal.command.position = 0.5;
      goal.command.max_effort = 10;
      gripper_.sendGoal(goal);
      bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(3));
      if (!finishedBeforeTimeout) {
        ROS_WARN("gripper_ open action did not complete");
        return false;
      }
      ROS_INFO("Grasped");

      return true;
    }

    bool PostGrasp(ros::NodeHandle& node_handle) {
      ROS_INFO("Moving to PostGrasped pose");

      ROS_INFO("PostGrasped Pose position = %f, %f, %f orientation = %f, %f, %f, %f",
          approaced_pose_.pose.position.x,
          approaced_pose_.pose.position.y,
          approaced_pose_.pose.position.z,
          approaced_pose_.pose.orientation.x,
          approaced_pose_.pose.orientation.y,
          approaced_pose_.pose.orientation.z,
          approaced_pose_.pose.orientation.w
          );
      double roll, pitch, yaw;
      GetRPY(approaced_pose_.pose.orientation, roll, pitch, yaw);
      ROS_INFO("postgrasped roll, pitch, yaw = %f, %f, %f", (float)roll, (float)pitch, (float)yaw);

      pub_arm_cartesian_.publish(approaced_pose_);

      ROS_INFO("Moved to PostGrasped pose");

      std::vector<std::string> start_controller;
      start_controller.push_back("xarm6_traj_controller");
      std::vector<std::string> stop_controller;
      stop_controller.push_back("xarm6_cartesian_motion_controller");

      ros::Duration(6).sleep();

      if (false == SwitchController(node_handle, start_controller, stop_controller)) {
        return false;
      }

      return true;
    }

    bool PostGraspVelocity(ros::NodeHandle& node_handle) {
      ROS_INFO("Moving to PostGrasped pose");

      ROS_INFO("PostGrasped Pose position = %f, %f, %f orientation = %f, %f, %f, %f",
          approaced_pose_.pose.position.x,
          approaced_pose_.pose.position.y,
          approaced_pose_.pose.position.z,
          approaced_pose_.pose.orientation.x,
          approaced_pose_.pose.orientation.y,
          approaced_pose_.pose.orientation.z,
          approaced_pose_.pose.orientation.w
          );
      double roll, pitch, yaw;
      GetRPY(approaced_pose_.pose.orientation, roll, pitch, yaw);
      ROS_INFO("postgrasped roll, pitch, yaw = %f, %f, %f", (float)roll, (float)pitch, (float)yaw);

      if (!CartesianVelCtrlOnPosCtrl(approaced_pose_)) return false;

      ROS_INFO("Moved to PostGrasped pose");

      std::vector<std::string> start_controller;
      start_controller.push_back("xarm6_traj_controller");
      std::vector<std::string> stop_controller;
      stop_controller.push_back("xarm6_cartesian_motion_controller");

      ros::Duration(6).sleep();

      if (false == SwitchController(node_handle, start_controller, stop_controller)) {
        return false;
      }

      return true;
    }

    bool PickVelocity(ros::NodeHandle& node_handle) {
      std::vector<std::string> start_controller;
      start_controller.push_back("xarm6_cartesian_motion_controller_velocity");
      std::vector<std::string> stop_controller;
      stop_controller.push_back("xarm6_traj_controller_velocity");

      if (false == SwitchController(node_handle, start_controller, stop_controller)) {
        return false;
      }
      //ros::Duration(2).sleep();

      ROS_INFO("Moving to picking pose");
      geometry_msgs::PoseStamped target_pose;
      target_pose.header.frame_id = FIXED_FRAME;

      {
        std::lock_guard<std::mutex> lock(mtx_);

        target_pose.pose.position.x = target_pose_.position.x + 0.01;
        target_pose.pose.position.y = target_pose_.position.y;
        //target_pose.pose.position.z = target_pose_.position.z - 0.07;
        target_pose.pose.position.z = target_pose_.position.z - 0.06;

        target_pose.pose.orientation.x = target_pose_.orientation.x;
        target_pose.pose.orientation.y = target_pose_.orientation.y;
        target_pose.pose.orientation.z = target_pose_.orientation.z;
        target_pose.pose.orientation.w = target_pose_.orientation.w;
      }

      static geometry_msgs::PoseStamped diff[2];
      static geometry_msgs::PoseStamped integral;
      memset(&diff[0], 0, sizeof(geometry_msgs::PoseStamped));
      memset(&diff[1], 0, sizeof(geometry_msgs::PoseStamped));
      memset(&integral, 0, sizeof(geometry_msgs::PoseStamped));

      geometry_msgs::PoseStamped* p_target = &target_pose;
      geometry_msgs::PoseStamped ref;
      geometry_msgs::PoseStamped current_pose;;
      geometry_msgs::TransformStamped approached_ts;

      unsigned int count = 0;
      #define APPROACHED (0)
      #define GRASPING (1)
      #define GRASPED (2)
      #define POSTGRASPED (3)
      unsigned int status = APPROACHED;
      bool isGrasped = false;
      ros::Rate rate(50);
      while (ros::ok()) {
        try { // link_tcpの現座標を取得
          approached_ts = tfBuffer_.lookupTransform(FIXED_FRAME, "link_tcp", ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
          ROS_WARN("%s", ex.what());
          return false;
        }
        transformTFStampedToPoseStamped(approached_ts, current_pose);

        float abs = 0.0f;
        if ((abs = std::abs(p_target->pose.position.z - current_pose.pose.position.z)) < 0.005) {
          switch(status) {
          case APPROACHED:
            if (count > 50) {
              ROS_INFO("Start Grasp: abs = %f, target_z = %f, current_z = %f",
                  abs, p_target->pose.position.z, current_pose.pose.position.z);
              control_msgs::GripperCommandGoal goal;
              goal.command.position = 0.5;
              goal.command.max_effort = 10;
              gripper_.sendGoal(goal);

              status = GRASPING;
              count = 0;
            }
            break;
          case GRASPING:
            if (count > 50) {
              ROS_INFO("Grasped: abs = %f, target_z = %f, current_z = %f",
                  abs, p_target->pose.position.z, current_pose.pose.position.z);
              p_target = &approaced_pose_;
              status = GRASPED;
              count = 0;
            }
            break;
          case GRASPED:
            if (count > 100) {
              ROS_INFO("Post Grasped: abs = %f, target_z = %f, current_z = %f",
                  abs, p_target->pose.position.z, current_pose.pose.position.z);
              ROS_INFO("break: abs = %f", abs);
              status = POSTGRASPED;
              break;
            }
          }

          if (status == POSTGRASPED) {
            break;
          }

          count++;
        } else {
          if (count > 0) count--;
        }

        ref.header = p_target->header;

        const float DELTA_T = 1.0/50.0;
        const float KP = 400;
        const float KP_R = 0;
        const float KI = 10;
        const float KI_R = 0;
        const float KD = 5;
        const float KD_R = 0;

        diff[0].pose.position.x = diff[1].pose.position.x;
        diff[0].pose.position.y = diff[1].pose.position.y;
        diff[0].pose.position.z = diff[1].pose.position.z;
        diff[0].pose.orientation.x = diff[1].pose.orientation.x;
        diff[0].pose.orientation.y = diff[1].pose.orientation.y;
        diff[0].pose.orientation.z = diff[1].pose.orientation.z;
        diff[0].pose.orientation.w = diff[1].pose.orientation.w;
        diff[1].pose.position.x = p_target->pose.position.x - current_pose.pose.position.x;
        diff[1].pose.position.y = p_target->pose.position.y - current_pose.pose.position.y;
        diff[1].pose.position.z = p_target->pose.position.z - current_pose.pose.position.z;
        diff[1].pose.orientation.z = p_target->pose.orientation.x - current_pose.pose.orientation.x;
        diff[1].pose.orientation.y = p_target->pose.orientation.y - current_pose.pose.orientation.y;
        diff[1].pose.orientation.z = p_target->pose.orientation.z - current_pose.pose.orientation.z;
        diff[1].pose.orientation.w = p_target->pose.orientation.w - current_pose.pose.orientation.w;

        integral.pose.position.x += (diff[1].pose.position.x + diff[0].pose.position.x) / 2.0 * DELTA_T;
        integral.pose.position.y += (diff[1].pose.position.y + diff[0].pose.position.y) / 2.0 * DELTA_T;
        integral.pose.position.z += (diff[1].pose.position.z + diff[0].pose.position.z) / 2.0 * DELTA_T;
        integral.pose.orientation.x += (diff[1].pose.orientation.x + diff[0].pose.orientation.x) / 2.0 * DELTA_T;
        integral.pose.orientation.y += (diff[1].pose.orientation.y + diff[0].pose.orientation.y) / 2.0 * DELTA_T;
        integral.pose.orientation.z += (diff[1].pose.orientation.z + diff[0].pose.orientation.z) / 2.0 * DELTA_T;
        integral.pose.orientation.w += (diff[1].pose.orientation.w + diff[0].pose.orientation.w) / 2.0 * DELTA_T;

        ref.pose.position.x
          = p_target->pose.position.x
          + KP * diff[1].pose.position.x
          + KI * integral.pose.position.x
          + KD * (diff[1].pose.position.x - diff[0].pose.position.x) / DELTA_T;
        ref.pose.position.y
          = p_target->pose.position.y
          + KP * diff[1].pose.position.y
          + KI * integral.pose.position.y
          + KD * (diff[1].pose.position.y - diff[0].pose.position.y) / DELTA_T;
        ref.pose.position.z
          = p_target->pose.position.z
          + KP * diff[1].pose.position.z
          + KI * integral.pose.position.z
          + KD * (diff[1].pose.position.z - diff[0].pose.position.z) / DELTA_T;
        ref.pose.orientation.x
          = p_target->pose.orientation.x
          + KP_R * diff[1].pose.orientation.x
          + KI_R * integral.pose.orientation.x
          + KD_R * (diff[1].pose.orientation.x - diff[0].pose.orientation.x) / DELTA_T;
        ref.pose.orientation.y
          = p_target->pose.orientation.y
          + KP_R * diff[1].pose.orientation.y
          + KI_R * integral.pose.orientation.y
          + KD_R * (diff[1].pose.orientation.y - diff[0].pose.orientation.y) / DELTA_T;
        ref.pose.orientation.z
          = p_target->pose.orientation.z
          + KP_R * diff[1].pose.orientation.z
          + KI_R * integral.pose.orientation.z
          + KD_R * (diff[1].pose.orientation.z - diff[0].pose.orientation.z) / DELTA_T;
        ref.pose.orientation.w
          = p_target->pose.orientation.w
          + KP_R * diff[1].pose.orientation.w
          + KI_R * integral.pose.orientation.w
          + KD_R * (diff[1].pose.orientation.w - diff[0].pose.orientation.w) / DELTA_T;

        pub_arm_cartesian_vel_.publish(ref);

        rate.sleep();
      }

      ROS_INFO("Moved to picking pose");

      start_controller.clear();
      start_controller.push_back("xarm6_traj_controller_velocity");
      stop_controller.clear();
      stop_controller.push_back("xarm6_cartesian_motion_controller_velocity");

      ros::Duration(6).sleep();

      if (false == SwitchController(node_handle, start_controller, stop_controller)) {
        return false;
      }

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
      // TODO カメラのyawとGripperのyawが90度ずれている
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

#if 0
      std::cout << "target_pose.pose = " << approaching_pose.pose << std::endl;
      approaching_pose.pose.orientation.x = 0.0f;
      approaching_pose.pose.orientation.y = 1.0f;
      approaching_pose.pose.orientation.z = 0.0f;
      approaching_pose.pose.orientation.w = 0.0f;
#endif
      {
        std::lock_guard<std::mutex> lock(mtx_);

        target_pose_.position = approaching_pose.pose.position;
        target_pose_.orientation = approaching_pose.pose.orientation;
      }
    }

  private:
#define CAR_VEL_CTL_LOG
    bool CartesianVelCtrlOnPosCtrl(geometry_msgs::PoseStamped target_pose) {
      geometry_msgs::Point velo_point;
      geometry_msgs::Vector3 rol_vec3;
      double tgt_roll, tgt_pitch, tgt_yaw;
      double cur_roll, cur_pitch, cur_yaw;
      double vel_roll, vel_pitch, vel_yaw;
      double nxt_roll, nxt_pitch, nxt_yaw;

      geometry_msgs::PoseStamped current_pose;;
      geometry_msgs::TransformStamped approached_ts;
      try { // target_linkの現座標を取得
        approached_ts = tfBuffer_.lookupTransform(FIXED_FRAME, CAR_CTL_VEL_TARGET_LINK,
            ros::Time(0), ros::Duration(1.0));
      } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
      }
      transformTFStampedToPoseStamped(approached_ts, current_pose);

      velo_point.x = (target_pose.pose.position.x - current_pose.pose.position.x)
        / std::abs(target_pose.pose.position.x - current_pose.pose.position.x) * CAR_CTL_VEL_P;
      velo_point.y = (target_pose.pose.position.y - current_pose.pose.position.y)
        / std::abs(target_pose.pose.position.y - current_pose.pose.position.y) * CAR_CTL_VEL_P;
      velo_point.z = (target_pose.pose.position.z - current_pose.pose.position.z)
        / std::abs(target_pose.pose.position.z - current_pose.pose.position.z) * CAR_CTL_VEL_P;

      GetRPY(target_pose.pose.orientation, tgt_roll, tgt_pitch, tgt_yaw);
      GetRPY(current_pose.pose.orientation, cur_roll, cur_pitch, cur_yaw);

      vel_roll  = (tgt_roll  - cur_roll ) / std::abs(tgt_roll  - cur_roll ) * CAR_CTL_VEL_R;
      vel_pitch = (tgt_pitch - cur_pitch) / std::abs(tgt_pitch - cur_pitch) * CAR_CTL_VEL_R;
      vel_yaw   = (tgt_yaw   - cur_yaw  ) / std::abs(tgt_yaw   - cur_yaw  ) * CAR_CTL_VEL_R;

      geometry_msgs::PoseStamped next_pose;
      ros::Time now = ros::Time::now();
      ros::Time prev;
      uint32_t seq = 0;
      ros::Rate rate(CAR_CTL_DURATION);
#if defined(CAR_VEL_CTL_LOG)
      unsigned int cnt = 0;
#endif
      while (ros::ok()) {
        uint32_t finished = 0;
        float diff;
        float delta_p;
        double delta_r;
        prev = now;
        now = ros::Time::now();
        ros::Duration duration = now - prev;

        try { // target_linkの現座標を取得
          approached_ts = tfBuffer_.lookupTransform(FIXED_FRAME, CAR_CTL_VEL_TARGET_LINK,
              ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
          ROS_WARN("%s", ex.what());
          return false;
        }
        transformTFStampedToPoseStamped(approached_ts, current_pose);

        next_pose.header.seq = seq++;
        next_pose.header.stamp = now;
        next_pose.header.frame_id = FIXED_FRAME;

        vel_roll  = std::abs(tgt_roll  - cur_roll ) > M_PI ? -vel_roll  : vel_roll;
        vel_pitch = std::abs(tgt_pitch - cur_pitch) > M_PI ? -vel_pitch : vel_pitch;
        vel_yaw   = std::abs(tgt_yaw   - cur_yaw  ) > M_PI ? -vel_yaw   : vel_yaw;

        delta_p = velo_point.x * duration.toSec();
        if (std::abs(target_pose.pose.position.x - current_pose.pose.position.x) > std::abs(delta_p)) {
          next_pose.pose.position.x = current_pose.pose.position.x + delta_p;
        } else {
          next_pose.pose.position.x = target_pose.pose.position.x;
          //finished++;
          finished += 1;
        }
        delta_p = velo_point.y * duration.toSec();
        if (std::abs(target_pose.pose.position.y - current_pose.pose.position.y) > std::abs(delta_p)) {
          next_pose.pose.position.y = current_pose.pose.position.y + delta_p;
        } else {
          next_pose.pose.position.y = target_pose.pose.position.y;
          //finished++;
          finished += 10;
        }
        delta_p = velo_point.z * duration.toSec();
        if (std::abs(target_pose.pose.position.z - current_pose.pose.position.z) > std::abs(delta_p)) {
          next_pose.pose.position.z = current_pose.pose.position.z + delta_p;
        } else {
          next_pose.pose.position.z = target_pose.pose.position.z;
          //finished++;
          finished += 100;
        }

        GetRPY(target_pose.pose.orientation, tgt_roll, tgt_pitch, tgt_yaw);
        GetRPY(current_pose.pose.orientation, cur_roll, cur_pitch, cur_yaw);

        delta_r = vel_roll * duration.toSec();
        if ((diff = std::abs(tgt_roll - cur_roll)) > M_PI) diff = std::abs((tgt_roll + 2*M_PI) - cur_roll );
        if (diff > std::abs(delta_r)) {
          nxt_roll = cur_roll + delta_r;
          if (nxt_roll >  M_PI) -M_PI + (nxt_roll - M_PI);
          if (nxt_roll < -M_PI)  M_PI + (nxt_roll + M_PI);
        } else {
          nxt_roll = tgt_roll;
          //finished++;
          finished += 1000;
        }
        delta_r = vel_pitch * duration.toSec();
        if ((diff = std::abs(tgt_pitch - cur_pitch)) > M_PI) diff = std::abs((tgt_pitch + 2*M_PI) - cur_pitch );
        if (diff > std::abs(delta_r)) {
          nxt_pitch = cur_pitch + delta_r;
          if (nxt_pitch >  M_PI) -M_PI + (nxt_pitch - M_PI);
          if (nxt_pitch < -M_PI)  M_PI + (nxt_pitch + M_PI);
        } else {
          nxt_pitch = tgt_pitch;
          //finished++;
          finished += 10000;
        }
        delta_r = vel_yaw * duration.toSec();
        if ((diff = std::abs(tgt_yaw - cur_yaw)) > M_PI) diff = std::abs((tgt_yaw + 2*M_PI) - cur_yaw );
        if (diff > std::abs(delta_r)) {
          nxt_yaw = cur_yaw + delta_r;
          if (nxt_yaw >  M_PI) -M_PI + (nxt_yaw - M_PI);
          if (nxt_yaw < -M_PI)  M_PI + (nxt_yaw + M_PI);
        } else {
          nxt_yaw = tgt_yaw;
          //finished++;
          finished += 100000;
        }

        GetQuaternionMsg(nxt_roll, nxt_pitch, nxt_yaw, next_pose.pose.orientation);

        pub_arm_cartesian_.publish(next_pose);

#if defined(CAR_VEL_CTL_LOG)
        if (cnt % CAR_CTL_DURATION == 0) {
          std::cout << finished << std::endl;
          std::cout << "duration : " << duration.toSec() << std::endl;

          std::cout << "current  : ";
          std::cout << current_pose.pose.position.x;
          std::cout << ", ";
          std::cout << current_pose.pose.position.y;
          std::cout << ", ";
          std::cout << current_pose.pose.position.z;
          std::cout << ", ";
          std::cout << cur_roll;
          std::cout << ", ";
          std::cout << cur_pitch;
          std::cout << ", ";
          std::cout << cur_yaw;
          std::cout << std::endl;
          std::cout << "delta    : ";
          std::cout << velo_point.x * duration.toSec();
          std::cout << ", ";
          std::cout << velo_point.y * duration.toSec();
          std::cout << ", ";
          std::cout << velo_point.z * duration.toSec();
          std::cout << ", ";
          std::cout << vel_roll * duration.toSec();
          std::cout << ", ";
          std::cout << vel_pitch * duration.toSec();
          std::cout << ", ";
          std::cout << vel_yaw * duration.toSec();
          std::cout << std::endl;
          std::cout << "next     : ";
          std::cout << next_pose.pose.position.x;
          std::cout << ", ";
          std::cout << next_pose.pose.position.y;
          std::cout << ", ";
          std::cout << next_pose.pose.position.z;
          std::cout << ", ";
          std::cout << nxt_roll;
          std::cout << ", ";
          std::cout << nxt_pitch;
          std::cout << ", ";
          std::cout << nxt_yaw;
          std::cout << std::endl;
          std::cout << "target   : ";
          std::cout << target_pose.pose.position.x;
          std::cout << ", ";
          std::cout << target_pose.pose.position.y;
          std::cout << ", ";
          std::cout << target_pose.pose.position.z;
          std::cout << ", ";
          std::cout << tgt_roll;
          std::cout << ", ";
          std::cout << tgt_pitch;
          std::cout << ", ";
          std::cout << tgt_yaw;
          std::cout << std::endl;
        }
        if (finished == 111111) break;
        cnt++;
#endif

        rate.sleep();
      }

      return true;
    }



    moveit::planning_interface::MoveGroupInterface arm_;
    moveit::planning_interface::PlanningSceneInterface scene_;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
    ros::Publisher pub_arm_cartesian_;
    ros::Publisher pub_arm_cartesian_vel_;
    ros::Publisher pub_marker_;
    geometry_msgs::Pose target_pose_;
    geometry_msgs::PoseStamped cognition_pose_;
    geometry_msgs::PoseStamped approaced_pose_;
    std::mutex mtx_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tflistener_;
    image_geometry::PinholeCameraModel cam_model_;

    const std::string PLANNING_GROUP = "xarm6";
    const std::string FIXED_FRAME = "world";
    const unsigned int CAR_CTL_DURATION = 50;
    const float CAR_CTL_VEL_P = 2.0;
    const float CAR_CTL_VEL_R = 2.0;
    const std::string CAR_CTL_VEL_TARGET_LINK = "link_tcp";
};

int main(int argc, char** argv)
{
  static const std::string MY_NODE_NAME = "xarm6_demo_app1_node";
  static const std::string PARAM_VELCTL = "/" + MY_NODE_NAME + "/velocity_control";
  ros::init(argc, argv, MY_NODE_NAME);
  ros::NodeHandle node_handle;
  // MoveIt!はアシンクロナスな計算をしないといけないので、このコードによりROSのアシンクロナスな機能を初期化する。
  ros::AsyncSpinner spinner(10);
  spinner.start();

  if(!node_handle.hasParam(PARAM_VELCTL))
  {
    ROS_ERROR("No velocity_control parameter specified!");
    exit(-1);
  }
  bool velctl = false;
  node_handle.getParam(PARAM_VELCTL, velctl);
  ROS_INFO("param velocity_control = %s", velctl ? "true" : "false");


  VisualServoTest pnp(node_handle);
  pnp.MoveToCognitionPose(node_handle);
  pnp.DoApproach(node_handle);
  if (velctl) {
    pnp.PickVelocity(node_handle);
  } else {
    pnp.PreGraspVelocity(node_handle);
    ros::Duration(5).sleep();
    pnp.Grasp(node_handle);
    pnp.PostGraspVelocity(node_handle);
  }

  //spinner.stop();
  // Wait until the node is shut down
  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}
