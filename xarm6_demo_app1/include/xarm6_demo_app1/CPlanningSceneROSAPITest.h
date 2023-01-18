#if !defined(XARM6_DEMO_APP1_CPLANNINGSCENEROSAPITEST_H)
#define XARM6_DEMO_APP1_CPLANNINGSCENEROSAPITEST_H

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_visual_tools/moveit_visual_tools.h>


class CPlanningSceneROSAPITest {
  public:
    explicit CPlanningSceneROSAPITest(ros::NodeHandle& node_handle, const std::string model_frame);
    bool Test();

  private:
    ros::NodeHandle& node_handle;
    moveit_visual_tools::MoveItVisualTools visual_tools;
    std::string robot_base_frame_;
};

#endif // XARM6_DEMO_APP1_CPLANNINGSCENEROSAPITEST_H
