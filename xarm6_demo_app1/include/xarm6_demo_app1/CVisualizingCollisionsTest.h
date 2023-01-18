#if !defined(XARM6_DEMO_APP1_CVISUALIZINGCOLLISIONSTEST_H)
#define XARM6_DEMO_APP1_CVISUALIZINGCOLLISIONSTEST_H

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "interactivity/interactive_robot.h"


class CVisualizingCollisionsTest {
  public:
    explicit CVisualizingCollisionsTest(ros::NodeHandle& node_handle, const std::string model_frame, const std::string planning_group);
    void help();
    bool Test();

  private:
    ros::NodeHandle& node_handle_;
    moveit_visual_tools::MoveItVisualTools visual_tools_;
    std::string robot_base_frame_;
    std::string planning_group_;
    robot_model_loader::RobotModelLoader robot_model_loader_;

    friend void computeCollisionContactPoints(InteractiveRobot& robot);
    friend void publishMarkers(visualization_msgs::MarkerArray& markers);
};

#endif // XARM6_DEMO_APP1_CVISUALIZINGCOLLISIONSTEST_H

