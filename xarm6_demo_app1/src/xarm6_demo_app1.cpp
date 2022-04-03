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

#include <xarm6_demo_app1/CApproach.h>
#include <xarm6_demo_app1/CGrasp.h>
#include <xarm6_demo_app1/CObjListManager.h>
#include <xarm6_demo_app1/CMovingAveragePose.h>
#include <xarm6_demo_app1/Utility.h>

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

  CApproach aprch(node_handle);
  aprch.MoveToCognitionPose(node_handle);
  aprch.DoApproach(node_handle);
  aprch.DoApproachRotation(node_handle);

  CGrasp grasp(node_handle);
  if (velctl) {
    grasp.PickVelocity(node_handle);
  } else {
    grasp.PreGrasp(node_handle);
    //grasp.PreGraspVelocity(node_handle);
    //ros::Duration(5).sleep();
    grasp.Grasp(node_handle);
    grasp.PostGrasp(node_handle);
    //grasp.PostGraspVelocity(node_handle);
  }

  //spinner.stop();
  // Wait until the node is shut down
  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}
