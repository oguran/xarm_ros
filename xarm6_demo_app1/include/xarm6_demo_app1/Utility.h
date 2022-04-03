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

/**
 * @brief RPYからクオータニオンを取得する関数
 *
 * @param roll [rad]
 * @param pitch [rad]
 * @param yaw [rad]
 * @param[out] q クオータニオン
 */
inline void GetQuaternionMsg(
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
inline void GetRPY(const geometry_msgs::Quaternion &q,
    double &roll,double &pitch,double &yaw){
  tf::Quaternion quat(q.x,q.y,q.z,q.w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

inline void transformTFStampedToPoseStamped(
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

inline bool SwitchController(ros::NodeHandle &node_handle,
    const std::vector<std::string> start_controller,
    const std::vector<std::string> stop_controller) {

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

inline void copyPose(const geometry_msgs::Pose &src, geometry_msgs::Pose &dst) {
  dst.position = src.position;
  dst.orientation = src.orientation;
}

inline void printPose(std::string const& msg, const geometry_msgs::Pose &pose) {
  ROS_INFO("%s position = %f, %f, %f orientation = %f, %f, %f, %f",
      msg.c_str(),
      pose.position.x,
      pose.position.y,
      pose.position.z,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w
      );
}

inline void printEuler(std::string const& msg, float roll, float pitch, float yaw) {
  ROS_INFO("%s roll, pitch, yaw  = %f, %f, %f", msg.c_str(), (float)roll, (float)pitch, (float)yaw);
}

#if 0
void ShowControllerStatistics(ros::NodeHandle& node_handle,) {
  ros::ServiceClient controller_statistics =
    node_handle.serviceClient<controller_manager_msgs::ControllerStatistics>("xarm/controller_manager/list_controllers");
  controller_manager_msgs::ListControllers list_controllers;
  ros::service::waitForService("xarm/controller_manager/list_controllers", ros::Duration(5));
}
#endif



