#include <xarm6_demo_app1/CApproach.h>
#include <xarm6_demo_app1/CGrasp.h>
#include <xarm6_demo_app1/CObjListManager.h>
#include <xarm6_demo_app1/CMovingAveragePose.h>
#include <xarm6_demo_app1/CMoveGroupTest.h>
#include <xarm6_demo_app1/CRobotModelStateTest.h>
#include <xarm6_demo_app1/CPlanningSceneTest.h>
#include <xarm6_demo_app1/CPlanningSceneROSAPITest.h>
#include <xarm6_demo_app1/CMotionPlanningAPITest.h>
#include <xarm6_demo_app1/CMotionPlanningPipelineTest.h>
#include <xarm6_demo_app1/CVisualizingCollisionsTest.h>

extern void timer_callback(const ros::TimerEvent& e);

int main(int argc, char** argv)
{
  static const std::string MY_NODE_NAME = "xarm6_demo_app1_node";
  static const std::string PARAM_VELCTL = "/" + MY_NODE_NAME + "/velocity_control";
  static const std::string PLANNING_GROUP = "xarm6";
  ros::init(argc, argv, MY_NODE_NAME);
  ros::NodeHandle node_handle;
  // MoveIt!はアシンクロナスな計算をしないといけないので、このコードによりROSのアシンクロナスな機能を初期化する。
  ros::AsyncSpinner spinner(10);
  spinner.start();

  bool plan_confirm = true;

  if(!node_handle.hasParam(PARAM_VELCTL))
  {
    ROS_ERROR("No velocity_control parameter specified!");
    exit(-1);
  }
  bool velctl = false;
  node_handle.getParam(PARAM_VELCTL, velctl);
  ROS_INFO("param velocity_control = %s", velctl ? "true" : "false");

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model =  robot_model_loader.getModel();
  std::string robot_model_frame = kinematic_model->getModelFrame();
  ROS_INFO("Model frame: %s", robot_model_frame.c_str());

  CObjListManager olm(node_handle);

#if 0
  CApproach aprch(node_handle, olm, robot_model_frame, PLANNING_GROUP);
  aprch.MoveToHomePose(plan_confirm);
  aprch.MoveToCognitionPose(plan_confirm);

  ros::Duration(10).sleep();

  aprch.ObjPoseCognition();

  aprch.DoApproach(plan_confirm);

  ros::Duration(10).sleep();

  aprch.DoApproachRotation(plan_confirm);

  ros::Duration(10).sleep();

  CGrasp grasp(node_handle, olm, aprch);

  if (velctl) {
    grasp.PickVelocity();
  } else {
    grasp.PreGraspCartesian(CGrasp::E_CTRL_TYPE::Position);
    grasp.Grasp();
    grasp.PostGraspCartesian(CGrasp::E_CTRL_TYPE::Position);
  }

#else // Test Code

#if 0 // MoveGroup Test
  CMoveGroupTest mvtest(node_handle, olm);
  mvtest.MoveToHomePose(plan_confirm);
  mvtest.MoveToCognitionPose(plan_confirm);
  //mvtest.ConstraintTest(plan_confirm);
  /mvtest.CartesianPathsTest(plan_confirm);
  mvtest.AddRemoveAttachDetachObject(plan_confirm);
#endif

#if 0 // Robot Model State Test
  CRobotModelStateTest rmstest;
  rmstest.Test();
#endif

#if 0 // Planning Scene Test
  CPlanningSceneTest pstest;
  pstest.Test();
#endif

#if 0 // Planning Scene ROS API Test
  CPlanningSceneROSAPITest psratest(node_handle, robot_model_frame);
  psratest.Test();
#endif

#if 0 // Motion Plannning API Test
  CMotionPlanningAPITest mpapitest(node_handle, robot_model_frame, PLANNING_GROUP);
  mpapitest.Test();
#endif

#if 0 // Motion Planning Pipeline Test
  CMotionPlanningPipelineTest mpptest(node_handle, robot_model_frame, PLANNING_GROUP);
  mpptest.Test();
#endif

#if 1 // Visualizing Collisions Test
  CVisualizingCollisionsTest vctest(node_handle, robot_model_frame, PLANNING_GROUP);
  vctest.Test();
#endif
#endif

  //spinner.stop();
  // Wait until the node is shut down
  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}
