#include <xarm6_demo_app1/CPlanningSceneROSAPITest.h>
#include <xarm6_demo_app1/Utility.h>

// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>


CPlanningSceneROSAPITest::CPlanningSceneROSAPITest(ros::NodeHandle& node_handle, const std::string model_frame)
  : node_handle(node_handle), robot_base_frame_(model_frame), visual_tools(model_frame)
{
}


bool CPlanningSceneROSAPITest::Test() {
  // BEGIN_TUTORIAL
  //
  // Visualization
  // ^^^^^^^^^^^^^
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  visual_tools.deleteAllMarkers();

  // ROS API
  // ^^^^^^^
  // The ROS API to the planning scene publisher is through a topic interface
  // using "diffs". A planning scene diff is the difference between the current
  // planning scene (maintained by the move_group node) and the new planning
  // scene desired by the user.
  //
  // Advertise the required topic
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // We create a publisher and wait for subscribers
  // Note that this topic may need to be remapped in the launch file
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Define the attached object message
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // We will use this message to add or
  // subtract the object from the world
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "left_finger";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "left_finger";
  /* The id of the object */
  attached_object.object.id = "box";

  /* A default pose */
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  // Note that attaching an object to the robot requires
  // the corresponding operation to be specified as an ADD operation
  attached_object.object.operation = attached_object.object.ADD;

  // Since we are attaching the object to the robot hand to simulate picking up the object,
  // we want the collision checker to ignore collisions between the object and the robot hand
  attached_object.touch_links = std::vector<std::string>{ "xarm_gripper_base_link", "left_finger", "right_finger" };

  // Add an object into the environment
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Add the object into the environment by adding it to
  // the set of collision objects in the "world" part of the
  // planning scene. Note that we are using only the "object"
  // field of the attached_object message here.
  ROS_INFO("Adding the object into the world at the location of the hand.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Interlude: Synchronous vs Asynchronous updates
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // There are two separate mechanisms available to interact
  // with the move_group node using diffs:
  //
  // * Send a diff via a rosservice call and block until
  //   the diff is applied (synchronous update)
  // * Send a diff via a topic, continue even though the diff
  //   might not be applied yet (asynchronous update)
  //
  // While most of this tutorial uses the latter mechanism (given the long sleeps
  // inserted for visualization purposes asynchronous updates do not pose a problem),
  // it would is perfectly justified to replace the planning_scene_diff_publisher
  // by the following service client:
  ros::ServiceClient planning_scene_diff_client =
    node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();
  // and send the diffs to the planning scene via a service call:
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);
  // Note that this does not continue until we are sure the diff has been applied.
  //
  // Attach an object to the robot
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // When the robot picks up an object from the environment, we need to
  // "attach" the object to the robot so that any component dealing with
  // the robot model knows to account for the attached object, e.g. for
  // collision checking.
  //
  // Attaching an object requires two operations
  //  * Removing the original object from the environment
  //  * Attaching the object to the robot

  /* First, define the REMOVE object message*/
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = robot_base_frame_;
  remove_object.operation = remove_object.REMOVE;

  // Note how we make sure that the diff message contains no other
  // attached objects or collisions objects by clearing those fields
  // first.
  /* Carry out the REMOVE + ATTACH operation */
  ROS_INFO("Attaching the object to the hand and removing it from the world.");
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene_diff_publisher.publish(planning_scene);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Detach an object from the robot
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Detaching an object from the robot requires two operations
  //  * Detaching the object from the robot
  //  * Re-introducing the object into the environment

  /* First, define the DETACH object message*/
  moveit_msgs::AttachedCollisionObject detach_object;
  detach_object.object.id = "box";
  //detach_object.link_name = "panda_link8";
  detach_object.link_name = "link_tcp";
  detach_object.object.operation = attached_object.object.REMOVE;

  // Note how we make sure that the diff message contains no other
  // attached objects or collisions objects by clearing those fields
  // first.
  /* Carry out the DETACH + ADD operation */
  ROS_INFO("Detaching the object from the robot and returning it to the world.");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.robot_state.is_diff = true;
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Remove the object from the collision world
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // Removing the object from the collision world just requires
  // using the remove object message defined earlier.
  // Note, also how we make sure that the diff message contains no other
  // attached objects or collisions objects by clearing those fields
  // first.
  ROS_INFO("Removing the object from the world.");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene_diff_publisher.publish(planning_scene);
  // END_TUTORIAL

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");

  return true;
}
