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
#include <math.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/GripperCommandAction.h>

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


class PickNPlacer {
  public:
    explicit PickNPlacer(ros::NodeHandle& node_handle)
      : arm_("xarm6"),
      gripper_("/xarm/gripper_controller/gripper_cmd", "true") {
        // 座標系をロボットのベースに基づいた「base_link」座標系を使う。
        arm_.setPoseReferenceFrame("base_link");

        // MoveIt!は「Named pose」というコンセプトを持つ。MoveIt!コンフィギュレーションに記載された名前付きポーズを指定できる。
        ROS_INFO("Moving to home pose");
        arm_.setNamedTarget("home");
        // Targetを設定した後、移動命令を出す。
        arm_.move();

        gripper_.waitForServer();

        ros::Duration(2).sleep();

        ROS_INFO("Moving to cognition pose");
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.pose.position.x = 0.206873 + 0.25;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = 0.111828 + 0.25;
        pose.pose.orientation.x = 1.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 0.0;

        double roll, pitch, yaw;
        GetRPY(pose.pose.orientation, roll, pitch, yaw);
        ROS_INFO("roll = %f, pitch = %f, yaw = %f", roll, pitch, yaw);

        arm_.setPoseTarget(pose);
        if (!arm_.move()) {
          ROS_WARN("Could not move to cognition pose");
          return;
        }

        ros::Duration(2).sleep();

        ROS_INFO("Opening gripper");
        control_msgs::GripperCommandGoal goal;
        goal.command.position = 0.1;
        gripper_.sendGoal(goal);
        bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
        if (!finishedBeforeTimeout) {
          ROS_WARN("gripper_ open action did not complete");
          return;
        }

        sub_ = node_handle.subscribe("/block", 1, &PickNPlacer::DoPick, this);
        ROS_INFO("Subscribe prepared!");
    }

      void DoPick(geometry_msgs::Pose::ConstPtr const& msg) {

        ROS_INFO("Moving to reached pose");
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "base_link";
        pose.pose.position.x = msg->position.x;
        pose.pose.position.y = msg->position.y;
        pose.pose.position.z = msg->position.z;
        pose.pose.orientation.x = msg->orientation.x;
        pose.pose.orientation.y = msg->orientation.y;
        pose.pose.orientation.z = msg->orientation.z;
        pose.pose.orientation.w = msg->orientation.w;

        arm_.setPoseTarget(pose);
        ROS_INFO("Done setPoseTarget");
        if (!arm_.move()) {
          ROS_WARN("Could not move to prepare pose");
          return;
        }
        ROS_INFO("Done reaching");

        ros::Duration(2).sleep();

        ROS_INFO("Closing gripper");
        control_msgs::GripperCommandGoal goal;
        goal.command.position = 0.7;
        gripper_.sendGoal(goal);
        bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
        if (!finishedBeforeTimeout) {
          ROS_WARN("Gripper close action did not complete");
          return;
        }

        ros::Duration(2).sleep();

        ROS_INFO("Moving to home pose");
        arm_.setNamedTarget("home");
        // Targetを設定した後、移動命令を出す。
        arm_.move();

        ros::Duration(2).sleep();

        ROS_INFO("Opening gripper");
        goal.command.position = 0.1;
        gripper_.sendGoal(goal);
        finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(30));
        if (!finishedBeforeTimeout) {
          ROS_WARN("Gripper open action did not complete");
          return;
        }
      }

  private:
    moveit::planning_interface::MoveGroupInterface arm_;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
    ros::Subscriber sub_;
    const std::string PLANNING_GROUP = "xarm6";
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "xarm6_demo_app1_node");
  ros::NodeHandle node_handle;
  // MoveIt!はアシンクロナスな計算をしないといけないので、このコードによりROSのアシンクロナスな機能を初期化する。
  ros::AsyncSpinner spinner(2);
  spinner.start();

  PickNPlacer pnp(node_handle);
  // Wait until the node is shut down
  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}
