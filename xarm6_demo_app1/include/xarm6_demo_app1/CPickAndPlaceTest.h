#if !defined(XARM6_DEMO_APP1_CPICKANDPLACETEST_H)
#define XARM6_DEMO_APP1_CPICKANDPLACETEST_H

#include <string>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class CPickAndPlaceTest {
  public:
    explicit CPickAndPlaceTest(const std::string model_frame, const std::string planning_group);
    void openGripper(trajectory_msgs::JointTrajectory& posture);
    void closedGripper(trajectory_msgs::JointTrajectory& posture);
    void pick(moveit::planning_interface::MoveGroupInterface& move_group);
    void place(moveit::planning_interface::MoveGroupInterface& group);
    void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
    bool Test();

  private:
    std::string robot_base_frame_;
    std::string planning_group_;
};

#endif // XARM6_DEMO_APP1_CPICKANDPLACETEST_H
