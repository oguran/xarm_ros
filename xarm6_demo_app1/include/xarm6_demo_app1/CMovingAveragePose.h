#if !defined(XARM6_DEMO_APP1_CMOVINGAVERAGEPOSE_H)
#define XARM6_DEMO_APP1_CMOVINGAVERAGEPOSE_H

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

class CMovingAveragePose {
  public:
    explicit CMovingAveragePose (unsigned char num);
    void averagedPose(geometry_msgs::Pose &pose, geometry_msgs::Pose &avePose);
  private:
    std::unique_ptr<std::list<geometry_msgs::Pose>> listPose_;
    unsigned int denomitator_ = 0;
};

#endif // endifXARM6_DEMO_APP1_CMOVINGAVERAGEPOSE_H
