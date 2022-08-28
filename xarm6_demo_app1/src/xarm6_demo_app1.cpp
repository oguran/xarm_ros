#include <xarm6_demo_app1/CApproach.h>
#include <xarm6_demo_app1/CGrasp.h>
#include <xarm6_demo_app1/CObjListManager.h>
#include <xarm6_demo_app1/CMovingAveragePose.h>

extern void timer_callback(const ros::TimerEvent& e);

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

  ros::Duration(10).sleep();

  aprch.ObjPoseCognition();

  aprch.DoApproach();

  ros::Duration(10).sleep();

  aprch.DoApproachRotationTest();

  ros::Duration(10).sleep();

  CGrasp grasp(node_handle, olm, aprch);
  grasp.PreGraspCartesian();

#if 0
  ros::Rate rate(10);
  while (ros::ok()) {
    rate.sleep();
  }
#endif

#if 0
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
#endif

  //spinner.stop();
  // Wait until the node is shut down
  ros::waitForShutdown();

  ros::shutdown();
  return 0;
}
