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

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::Image, sensor_msgs::CameraInfo> DepthSyncPolicy;

class CApproach {
  public:
    explicit CApproach(ros::NodeHandle& node_handle);
    bool MoveToCognitionPose(ros::NodeHandle& node_handle);
    bool DoApproach(ros::NodeHandle& node_handle);
    bool DoApproachRotation(ros::NodeHandle& node_handle);

  private:
    moveit::planning_interface::MoveGroupInterface arm_;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
    ros::Publisher pub_marker_target_1st_;
    ros::Publisher pub_marker_target_2nd_;
    ros::Publisher pub_marker_target_3rd_;
    ros::Publisher pub_marker_target_rot_;
    geometry_msgs::Pose target_pose_;
    geometry_msgs::PoseStamped cognition_pose_;
    geometry_msgs::PoseStamped approaced_pose_;
    geometry_msgs::PoseStamped grasp_pose_[3]; // 0:pre-grasp, 1:grasp, 2:post-grasp
    std::mutex mtx_point_;
    tf2_ros::Buffer tfBuffer_;

    geometry_msgs::PoseStamped target_pose_1st_;

    const std::string FIXED_FRAME = "world";
    const std::string TARGET_FRAME = "target";
    const unsigned int PREGRASP_POSE = 0;
    const unsigned int GRASP_POSE = 1;
    const unsigned int POSTGRASP_POSE = 2;
};


