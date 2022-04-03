#include <xarm6_demo_app1/CApproach.h>
#include <xarm6_demo_app1/Utility.h>

CApproach::CApproach(ros::NodeHandle& node_handle)
    : arm_("xarm6"),
    gripper_("/xarm/gripper_controller/gripper_cmd", "true") {
        // 座標系をロボットのベースに基づいた「FIXED_FRAME」座標系を使う。
        arm_.setPoseReferenceFrame(FIXED_FRAME);
        gripper_.waitForServer();

        pub_marker_target_1st_ = node_handle.advertise<visualization_msgs::Marker>("marker_target_1st", 1);
        pub_marker_target_2nd_ = node_handle.advertise<visualization_msgs::Marker>("marker_target_2nd", 1);
        pub_marker_target_3rd_ = node_handle.advertise<visualization_msgs::Marker>("marker_target_3rd", 1);
        pub_marker_target_rot_= node_handle.advertise<visualization_msgs::Marker>("marker_target_rot", 1);
        ROS_INFO("Subscribe prepared!");
    }

bool CApproach::MoveToCognitionPose(ros::NodeHandle& node_handle) {
    ROS_INFO("Moving to cognition pose");
    cognition_pose_.header.frame_id = FIXED_FRAME;
    cognition_pose_.pose.position.x = -0.354;
    cognition_pose_.pose.position.y = -0.037;
    cognition_pose_.pose.position.z = 0.505;

    cognition_pose_.pose.orientation.x = 0.0;
    cognition_pose_.pose.orientation.y = 1.0;
    cognition_pose_.pose.orientation.z = 0.0;
    cognition_pose_.pose.orientation.w = 0.0;

    arm_.setPoseTarget(cognition_pose_);
    if (!arm_.move()) {
        ROS_WARN("Could not move to cognition pose");
        return false;
    }

    ROS_INFO("Initialized pose gripper");
    //control_msgs::GripperCommandGoal goal;
    //goal.command.position = 0.3;
    control_msgs::GripperCommandActionGoal goal;
    goal.goal.command.position = 0.3;
    gripper_.sendGoal(goal.goal);
    bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(3));
    if (!finishedBeforeTimeout) {
        ROS_WARN("gripper_ open action did not complete");
        return false;
    }
    ROS_INFO("Initialize gripper pose");

    return true;
}

bool CApproach::DoApproach(ros::NodeHandle& node_handle) {
    ROS_INFO("Opening gripper");
    control_msgs::GripperCommandGoal goal;
    goal.command.position = 0.0;
    gripper_.sendGoal(goal);
    bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(3));
    if (!finishedBeforeTimeout) {
        ROS_WARN("gripper_open action did not complete");
        return false;
    }

    ROS_INFO("Approaching");
    geometry_msgs::Pose pose;
    target_pose_1st_.header.frame_id = FIXED_FRAME;
#if 0
    ros::Rate rate(30);
    int cnt = 1;
    MovingAveragePose MAPose(cnt);
    while (ros::ok() && cnt--) {
        {
            std::lock_guard<std::mutex> lock(mtx_point_);
            copyPose(target_pose_, pose);
        }
        MAPose.averagedPose(pose, target_pose_1st_.pose);
        rate.sleep();
    }
#else
    {
        std::lock_guard<std::mutex> lock(mtx_point_);
        copyPose(target_pose_, target_pose_1st_.pose);
    }
#endif
    geometry_msgs::PoseStamped approached_pose;
    copyPose(target_pose_1st_.pose, approached_pose.pose);
    approached_pose.header.frame_id = target_pose_1st_.header.frame_id;
    approached_pose.pose.position.z += 0.10;

    printPose("link_eff", approached_pose.pose);
    double roll, pitch, yaw;
    GetRPY(approached_pose.pose.orientation, roll, pitch, yaw);
    printEuler("Approached", roll, pitch, yaw);

    arm_.setPoseTarget(approached_pose);
    if (!arm_.move()) {
        ROS_WARN("Could not approaching");
        return false;
    }

    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = target_pose_1st_.header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        copyPose(target_pose_1st_.pose, marker.pose);
        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;
        pub_marker_target_1st_.publish(marker);
    }

    ros::Duration(2).sleep();

    geometry_msgs::TransformStamped approached_ts;
    try { // link_tcpの現座標を取得
        approached_ts = tfBuffer_.lookupTransform(FIXED_FRAME, "link_tcp", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }
    transformTFStampedToPoseStamped(approached_ts, approaced_pose_);

    return true;
}

bool CApproach::DoApproachRotation(ros::NodeHandle& node_handle) {
    ROS_INFO("Rotation");
    geometry_msgs::PoseStamped l_grasp_pose[2];
    geometry_msgs::PoseStamped l_2nd_pose, g_2nd_pose;
    grasp_pose_[PREGRASP_POSE].header.frame_id = FIXED_FRAME;
    {
        std::lock_guard<std::mutex> lock(mtx_point_);
        copyPose(target_pose_, grasp_pose_[PREGRASP_POSE].pose);
    }
    grasp_pose_[PREGRASP_POSE].pose.position.z += 0.10;

    printPose("Global_based", grasp_pose_[PREGRASP_POSE].pose);

    double roll, pitch, yaw;
    GetRPY(grasp_pose_[PREGRASP_POSE].pose.orientation, roll, pitch, yaw);
    printEuler("Global_based", roll, pitch, yaw);

    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = grasp_pose_[PREGRASP_POSE].header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        copyPose(grasp_pose_[PREGRASP_POSE].pose, marker.pose);
        marker.color.r = 0.5f;
        marker.color.g = 0.5f;
        marker.color.b = 0.5f;
        marker.color.a = 1.0f;
        pub_marker_target_3rd_.publish(marker);
    }

    // 把持対象物を基準とした座標系に変換
    if (!tfBuffer_.canTransform(TARGET_FRAME, FIXED_FRAME, ros::Time(0), ros::Duration(10.0))) {
        ROS_WARN("Could not lookup transform from world to %s, in duration %f [sec]",
                TARGET_FRAME.c_str(), 10.0f);
        return false;
    }

    try {
        tfBuffer_.transform(grasp_pose_[PREGRASP_POSE], l_grasp_pose[PREGRASP_POSE], TARGET_FRAME, ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }

    printPose("Target_based", l_grasp_pose[PREGRASP_POSE].pose);
    GetRPY(l_grasp_pose[PREGRASP_POSE].pose.orientation, roll, pitch, yaw);
    printEuler("Target_based", roll, pitch, yaw);

    l_grasp_pose[GRASP_POSE].header.frame_id = TARGET_FRAME;
    copyPose(l_grasp_pose[PREGRASP_POSE].pose, l_grasp_pose[GRASP_POSE].pose);
    //l_grasp_pose[GRASP_POSE].pose.position.x += 0.01;
    //l_grasp_pose[GRASP_POSE].pose.position.z -= 0.06;
    l_grasp_pose[GRASP_POSE].pose.position.z += 0.14;

    l_2nd_pose.header.frame_id = TARGET_FRAME;
    copyPose(l_grasp_pose[GRASP_POSE].pose, l_2nd_pose.pose);

#if 0
    // Z axis (Yaw) rotation
    double theta_y = M_PI/2;
    l_grasp_pose[0].pose.position.x = l_grasp_pose[0].pose.position.x * cos(theta_y) - l_grasp_pose[0].pose.position.y * sin(theta_y);
    l_grasp_pose[0].pose.position.y = l_grasp_pose[0].pose.position.x * sin(theta_y) + l_grasp_pose[0].pose.position.y * cos(theta_y);
    yaw += theta_y;
    GetQuaternionMsg(roll, pitch, yaw, l_grasp_pose[0].pose.orientation);
#endif

#if 1
    // Y axis (Pitch) rotation
    double theta_p = M_PI/6;
    l_grasp_pose[PREGRASP_POSE].pose.position.x = l_grasp_pose[PREGRASP_POSE].pose.position.x * sin(theta_p) + l_grasp_pose[PREGRASP_POSE].pose.position.z * sin(theta_p);
    l_grasp_pose[GRASP_POSE].pose.position.x = l_grasp_pose[GRASP_POSE].pose.position.x * sin(theta_p) + l_grasp_pose[GRASP_POSE].pose.position.z * sin(theta_p);
    l_grasp_pose[PREGRASP_POSE].pose.position.z = -l_grasp_pose[PREGRASP_POSE].pose.position.x * sin(theta_p) + l_grasp_pose[PREGRASP_POSE].pose.position.z * cos(theta_p);
    //l_grasp_pose[GRASP_POSE].pose.position.z = -l_grasp_pose[GRASP_POSE].pose.position.x * sin(theta_p) + l_grasp_pose[GRASP_POSE].pose.position.z * cos(theta_p);
    // TODO tentative
    l_grasp_pose[GRASP_POSE].pose.position.z = -l_grasp_pose[GRASP_POSE].pose.position.x * sin(theta_p) + l_grasp_pose[GRASP_POSE].pose.position.z * cos(theta_p) + 0.035;
    pitch += theta_p;
    GetQuaternionMsg(roll, pitch, yaw, l_grasp_pose[PREGRASP_POSE].pose.orientation);
    GetQuaternionMsg(roll, pitch, yaw, l_grasp_pose[GRASP_POSE].pose.orientation);
#endif

#if 0
    // X axis (Roll) rotation
    double theta_r = M_PI/6;
    l_grasp_pose[0].pose.position.y = l_grasp_pose[0].pose.position.y * cos(theta_r) - l_grasp_pose[0].pose.position.z * sin(theta_r);
    l_grasp_pose[0].pose.position.z = l_grasp_pose[0].pose.position.y * sin(theta_r) + l_grasp_pose[0].pose.position.z * cos(theta_r);
    roll += theta_r;
    GetQuaternionMsg(roll, pitch, yaw, l_grasp_pose[0].pose.orientation);
#endif

    grasp_pose_[POSTGRASP_POSE].header.frame_id = target_pose_1st_.header.frame_id;
    //copyPose(approaced_pose_.pose, grasp_pose_[POSTGRASP_POSE].pose);
    copyPose(target_pose_1st_.pose, grasp_pose_[POSTGRASP_POSE].pose);

    // Global座標を基準とした座標系に変換
    if (!tfBuffer_.canTransform(FIXED_FRAME, TARGET_FRAME, ros::Time(0), ros::Duration(10.0))) {
        ROS_WARN("Could not lookup transform from world to %s, in duration %f [sec]",
                TARGET_FRAME.c_str(), 1.0f);
        return false;
    }

    try {
        tfBuffer_.transform(l_grasp_pose[PREGRASP_POSE], grasp_pose_[PREGRASP_POSE], FIXED_FRAME, ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }

    printPose("grasp_pose_[PREGRASP_POSE]", grasp_pose_[PREGRASP_POSE].pose);

    GetRPY(grasp_pose_[PREGRASP_POSE].pose.orientation, roll, pitch, yaw);
    printEuler("grasp_pose_[PREGRASP_POSE]", roll, pitch, yaw);

    try {
        tfBuffer_.transform(l_grasp_pose[GRASP_POSE], grasp_pose_[GRASP_POSE], FIXED_FRAME, ros::Duration(1.0));
        tfBuffer_.transform(l_2nd_pose, g_2nd_pose, FIXED_FRAME, ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }

    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = FIXED_FRAME;
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        copyPose(g_2nd_pose.pose, marker.pose);
        marker.color.r = 0.5f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        pub_marker_target_2nd_.publish(marker);
    }

    printPose("grasp_pose_[GRASP_POSE]", grasp_pose_[GRASP_POSE].pose);

    GetRPY(grasp_pose_[GRASP_POSE].pose.orientation, roll, pitch, yaw);
    printEuler("grasp_pose_[GRASP_POSE]", roll, pitch, yaw);

    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = grasp_pose_[GRASP_POSE].header.frame_id;
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;

        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration();

        marker.scale.x = 0.01;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        copyPose(grasp_pose_[GRASP_POSE].pose, marker.pose);
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        pub_marker_target_rot_.publish(marker);
    }

#if 1
    arm_.setPoseTarget(grasp_pose_[PREGRASP_POSE]);
    if (!arm_.move()) {
        ROS_WARN("Could not rotation");
        return false;
    }

    ros::Duration(2).sleep();

    geometry_msgs::TransformStamped approached_ts;
    try { // link_tcpの現座標を取得
        approached_ts = tfBuffer_.lookupTransform(FIXED_FRAME, "link_tcp", ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }
    transformTFStampedToPoseStamped(approached_ts, approaced_pose_);
#endif

    return true;
}

