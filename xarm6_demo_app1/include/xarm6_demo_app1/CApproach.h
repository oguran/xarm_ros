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

#include <xarm6_demo_app1/CObjListManager.h>

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::Image, sensor_msgs::CameraInfo> DepthSyncPolicy;

class CApproach {
  public:
    explicit CApproach(ros::NodeHandle& node_handle, CObjListManager& olm);
    bool MoveToCognitionPose();
    bool ObjPoseCognition();
    bool DoApproach();
    bool DoApproachRotationTest();
    bool DoApproachRotation();

    const unsigned int PREGRASP_POSE = 0;
    const unsigned int GRASP_POSE = 1;
    const unsigned int POSTGRASP_POSE = 2;

    geometry_msgs::PoseStamped approaced_pose_;
    geometry_msgs::PoseStamped grasp_pose_[3]; // 0:pre-grasp, 1:grasp, 2:post-grasp
    geometry_msgs::PoseStamped target_pose_1st_;

  private:
    moveit::planning_interface::MoveGroupInterface arm_;
    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
    ros::Publisher pub_marker_target_1st_;
    ros::Publisher pub_marker_target_2nd_;
    ros::Publisher pub_marker_target_3rd_;
    ros::Publisher pub_marker_target_rot_;
    geometry_msgs::PoseStamped cognition_pose_;
    geometry_msgs::PoseStamped target_obj_pose_local_;

    const std::string FIXED_FRAME = "world";
    const std::string TARGET_FRAME = "target";
    const std::string TARGET_OBJ_FRAME = "posecnn/00_potted_meat_can_01";
    ros::NodeHandle& node_handle;
    CObjListManager& olm;
};

#endif // XARM6_DEMO_APP1_CAPPROACH_H
