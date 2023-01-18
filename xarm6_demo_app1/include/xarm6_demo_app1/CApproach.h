#if !defined(XARM6_DEMO_APP1_CAPPROACH_H)
#define XARM6_DEMO_APP1_CAPPROACH_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/buffer_interface.h>
#include <control_msgs/GripperCommandAction.h>
#include <mutex>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

#include <xarm6_demo_app1/CObjListManager.h>

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::Image, sensor_msgs::CameraInfo> DepthSyncPolicy;

namespace rvt = rviz_visual_tools;

class CApproach {
  public:
    explicit CApproach(ros::NodeHandle& node_handle, CObjListManager& olm, std::string model_frame, std::string planning_group);
    bool MoveToHomePose(bool plan_confirm);
    bool MoveToCognitionPose(bool plan_confirm);
    bool ObjPoseCognition();
    bool DoApproach(bool plan_confirm);
    bool DoApproachRotation(bool plan_confirm);

    const unsigned int PREGRASP_POSE = 0;
    const unsigned int GRASP_POSE = 1;
    const unsigned int POSTGRASP_POSE = 2;

    const float PREGRASP_DISTANCE = 0.20f;

    geometry_msgs::PoseStamped approached_link_eef_pose_;
    geometry_msgs::PoseStamped approached_link_tcp_pose_;
    geometry_msgs::PoseStamped grasp_pose_[3]; // 0:pre-grasp, 1:grasp, 2:post-grasp
    geometry_msgs::PoseStamped target_pose_1st_;

    moveit::planning_interface::MoveGroupInterface arm_;

  private:
    const std::string TARGET_FRAME = "target";
    const std::string COGNITION_POSE = "cognition_pose";
    const std::string NAMED_POSE_HOME = "home";

    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
    ros::Publisher pub_marker_target_1st_;
    ros::Publisher pub_marker_target_2nd_;
    ros::Publisher pub_marker_target_3rd_;
    ros::Publisher pub_marker_target_rot_;
    geometry_msgs::PoseStamped cognition_pose_;
    geometry_msgs::PoseStamped target_obj_pose_local_;
    tf2_ros::TransformBroadcaster tf_bc_;
    moveit_visual_tools::MoveItVisualTools visual_tools;
    Eigen::Isometry3d text_pose;
    const robot_state::JointModelGroup* joint_model_group;
    ros::NodeHandle& node_handle;
    CObjListManager& olm;

    std::string robot_base_frame_;
};

#endif // XARM6_DEMO_APP1_CAPPROACH_H
