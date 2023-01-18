#if !defined(XARM6_DEMO_APP1_CROBOTMODEDSTATETEST_H)
#define XARM6_DEMO_APP1_CROBOTMODEDSTATETEST_H

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

class CRobotModelStateTest {
  public:
    explicit CRobotModelStateTest();
    bool Test();

  private:
    const std::string PLANNING_GROUP = "xarm6";
    robot_model_loader::RobotModelLoader robot_model_loader;
};

#endif // XARM6_DEMO_APP1_CROBOTMODEDSTATETEST_H
