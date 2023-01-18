#if !defined(XARM6_DEMO_APP1_CMOVEGROUPTEST_H)
#define XARM6_DEMO_APP1_CMOVEGROUPTEST_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2_ros/buffer_interface.h>
#include <mutex>
#include <eigen_stl_containers/eigen_stl_vector_container.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <xarm6_demo_app1/CObjListManager.h>

typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseStamped, sensor_msgs::Image, sensor_msgs::CameraInfo> DepthSyncPolicy;

namespace rvt = rviz_visual_tools;

class CMoveGroupTest {
  public:
    explicit CMoveGroupTest(ros::NodeHandle& node_handle, CObjListManager& olm);
    bool MoveToHomePose(bool plan_confirm);
    bool MoveToCognitionPose(bool plan_confirm);
    bool ConstraintTest(bool plan_confirm);
    bool CartesianPathsTest(bool plan_confirm);
    bool AddRemoveAttachDetachObject(bool plan_confirm);

  private:
    const std::string FIXED_FRAME = "world";
    const std::string TARGET_FRAME = "target";
    const std::string PLANNING_GROUP = "xarm6";
    const std::string COGNITION_POSE = "cognition_pose";
    const std::string NAMED_POSE_HOME = "home";

    moveit::planning_interface::MoveGroupInterface arm_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    geometry_msgs::PoseStamped cognition_pose_;
    moveit_visual_tools::MoveItVisualTools visual_tools;
    Eigen::Isometry3d text_pose;
    const robot_state::JointModelGroup* joint_model_group;
    ros::NodeHandle& node_handle;
    CObjListManager& olm;

    //control_msgs::FollowJointTrajectoryActionPtr traj_client_;
    //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
};

#endif // XARM6_DEMO_APP1_CMOVEGROUPTEST_HJointTrajectoryAction
