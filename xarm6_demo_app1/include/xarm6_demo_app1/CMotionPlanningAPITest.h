#if !defined(XARM6_DEMO_APP1_CMOTIONPLANNINGAPITEST_H)
#define XARM6_DEMO_APP1_CMOTIONPLANNINGAPITEST_H

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


class CMotionPlanningAPITest {
  public:
    explicit CMotionPlanningAPITest(ros::NodeHandle& node_handle, const std::string model_frame, const std::string planning_group);
    bool Test();

  private:
    ros::NodeHandle& node_handle_;
    moveit_visual_tools::MoveItVisualTools visual_tools_;
    std::string robot_base_frame_;
    std::string planning_group_;
    robot_model_loader::RobotModelLoader robot_model_loader_;
};

#endif // XARM6_DEMO_APP1_CMOTIONPLANNINGAPITEST_H
