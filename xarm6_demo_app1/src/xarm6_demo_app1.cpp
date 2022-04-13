#include <xarm6_demo_app1/CApproach.h>
#include <xarm6_demo_app1/CGrasp.h>
#include <xarm6_demo_app1/CObjListManager.h>
#include <xarm6_demo_app1/CMovingAveragePose.h>
#include <xarm6_demo_app1/Utility.h>

int main(int argc, char** argv)
{
  static const std::string MY_NODE_NAME = "xarm6_demo_app1_node";
  static const std::string PARAM_VELCTL = "/" + MY_NODE_NAME + "/velocity_control";
  ros::init(argc, argv, MY_NODE_NAME);
  ros::NodeHandle node_handle;
  // MoveIt!はアシンクロナスな計算をしないといけないので、このコードによりROSのアシンクロナスな機能を初期化する。
  ros::AsyncSpinner spinner(10);
  spinner.start();

  if(!node_handle.hasParam(PARAM_VELCTL))
  {
    ROS_ERROR("No velocity_control parameter specified!");
    exit(-1);
  }
  bool velctl = false;
  node_handle.getParam(PARAM_VELCTL, velctl);
  ROS_INFO("param velocity_control = %s", velctl ? "true" : "false");

  CObjListManager olm(node_handle);

  CApproach aprch(node_handle, olm);
  aprch.MoveToCognitionPose();
#if 1
  aprch.DoApproach();
  aprch.DoApproachRotation();

  CGrasp grasp(node_handle, olm, aprch);
  if (velctl) {
    grasp.PickVelocity();
  } else {
    grasp.PreGrasp();
    //grasp.PreGraspVelocity();
    //ros::Duration(5).sleep();
    grasp.Grasp();
    grasp.PostGrasp();
    //grasp.PostGraspVelocity();
  }
#else
  while (1) {
    ros::Duration(5).sleep();
  }
#endif

  //spinner.stop();
  // Wait until the node is shut down
  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}
