#if !defined(XARM6_DEMO_APP1_CGRASP_H)
#define XARM6_DEMO_APP1_CGRASP_H

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

#include <xarm6_demo_app1/CObjListManager.h>
#include <xarm6_demo_app1/CApproach.h>

class CGrasp {
  public:
    enum class E_CTRL_TYPE
    {
      Position,
      Velocity,
    };

    explicit CGrasp(ros::NodeHandle& node_handle, CObjListManager& olm, CApproach& aprch);
    bool PreGraspCartesian(E_CTRL_TYPE ctrl_type);
    bool Grasp();
    bool PostGraspCartesian(E_CTRL_TYPE ctrl_type);
    bool PickVelocity();

  private:
    bool CartesianVelCtrlOnPosCtrl(geometry_msgs::PoseStamped target_pose);


    actionlib::SimpleActionClient<control_msgs::GripperCommandAction> gripper_;
    ros::Publisher pub_arm_cartesian_;
    ros::Publisher pub_arm_cartesian_vel_;
    ros::Publisher pub_marker_target_grasp_;

    geometry_msgs::PoseStamped target_pose_1st_;

    const std::string FIXED_FRAME = "world";
    const unsigned int CAR_CTL_DURATION = 50;
    const float CAR_CTL_VEL_P = 2.0;
    const float CAR_CTL_VEL_R = 2.0;
    const std::string CAR_CTL_EEF_LINK = "link_tcp";
    const double GRASP_OFFSET = 0.05; //< Graspする際にlink_tcpからカメラ座標のz+方向にオフセットさせる距離

    ros::NodeHandle& node_handle;
    CObjListManager& olm;
    CApproach& aprch;
};

#endif // XARM6_DEMO_APP1_CGRASP_H
