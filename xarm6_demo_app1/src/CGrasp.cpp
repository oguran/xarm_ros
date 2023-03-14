#include <xarm6_demo_app1/CGrasp.h>
#include <xarm6_demo_app1/Utility.h>

#define XARM_GRIPPER

CGrasp::CGrasp(ros::NodeHandle& node_handle, CObjListManager& olm, CApproach& aprch)
    : gripper_("/xarm/gripper_controller/gripper_cmd", "true"),
    xarm_gripper_("xarm/gripper_move", "true"),
    node_handle(node_handle),
    olm(olm),
    aprch(aprch) {
        // 座標系をロボットのベースに基づいた「FIXED_FRAME」座標系を使う。
#if defined(XARM_GRIPPER)
	    xarm_gripper_.waitForServer();
#else
	    gripper_.waitForServer();
#endif
        pub_arm_cartesian_ = node_handle.advertise<geometry_msgs::PoseStamped>("/xarm/xarm6_cartesian_motion_controller/goal", 1);
        pub_arm_cartesian_vel_ = node_handle.advertise<geometry_msgs::PoseStamped>("/xarm/xarm6_cartesian_motion_controller_velocity/goal", 1);
        pub_marker_target_grasp_= node_handle.advertise<visualization_msgs::Marker>("marker_target_grasp", 1);
        ROS_INFO("Subscribe prepared!");
    }

bool CGrasp::PreGraspCartesian(E_CTRL_TYPE ctrl_type) {
  std::vector<std::string> start_controller;
  start_controller.push_back("xarm6_cartesian_motion_controller");
  std::vector<std::string> stop_controller;
  stop_controller.push_back("xarm6_traj_controller");

  if (false == SwitchController(node_handle, start_controller, stop_controller)) {
    return false;
  }
  ros::Duration(2).sleep();

  ROS_INFO("Moving to grasp pose");

  geometry_msgs::PoseStamped target_obj_pose_camera;
  CObjListManager::CObjPoint target_obj_point;

  {
    std::lock_guard<std::mutex> lock(olm.mtx_point_);
    if (olm.vect_target_obj_point_camera_.empty()) return false;
    target_obj_point.class_name_ = olm.vect_target_obj_point_camera_.at(0).class_name_;
    target_obj_point.header_ = olm.vect_target_obj_point_camera_.at(0).header_;
    target_obj_point.center_ = olm.vect_target_obj_point_camera_.at(0).center_;
    target_obj_point.left_ = olm.vect_target_obj_point_camera_.at(0).left_;
    target_obj_point.right_ = olm.vect_target_obj_point_camera_.at(0).right_;
  }

  geometry_msgs::TransformStamped tfs_linktcp_on_camera_frame;
  geometry_msgs::PoseStamped ps_goal_on_camera_frame, ps_goal_on_fixed_frame;

  try { // target_linkのcamera座標系を基準とした現座標を取得
    tfs_linktcp_on_camera_frame = olm.tfBuffer_.lookupTransform(target_obj_point.header_.frame_id, CAR_CTL_EEF_LINK,
        ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  transformTFStampedToPoseStamped(tfs_linktcp_on_camera_frame, ps_goal_on_camera_frame);

  // 座標をObjPointの値に上書き
  ps_goal_on_camera_frame.pose.position = target_obj_point.center_;
  // グリッパの間に入れるために、カメラ座標の奥方向(z+)に10cmずらす
  ps_goal_on_camera_frame.pose.position.z += GRASP_OFFSET;

  // カメラ座標系 -> ロボット基準座標系に変換
  if (!olm.tfBuffer_.canTransform(FIXED_FRAME, ps_goal_on_camera_frame.header.frame_id, ros::Time(0), ros::Duration(10.0))) {
    ROS_WARN("Could not lookup transform from %s to %s, in duration %f [sec]",
        ps_goal_on_camera_frame.header.frame_id.c_str(),
        FIXED_FRAME.c_str(),
        10.0f);
    return false;
  }

  try {
    olm.tfBuffer_.transform(ps_goal_on_camera_frame, ps_goal_on_fixed_frame, FIXED_FRAME, ros::Duration(10.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }

  // Catesian制御でPreGrasp実行
  if (ctrl_type == E_CTRL_TYPE::Position) {
    pub_arm_cartesian_.publish(ps_goal_on_fixed_frame);
  } else if (ctrl_type == E_CTRL_TYPE::Velocity) {
    if (!CartesianVelCtrlOnPosCtrl(ps_goal_on_fixed_frame)) return false;
  } else {
    ROS_WARN("elegal control type.");
    return false;
  }
  ros::Duration(10).sleep();
  //ros::Duration(50).sleep();
  ROS_INFO("Moved to grasping pose");

  return true;
}

bool CGrasp::Grasp() {

  ROS_INFO("Start Grasp");
#if defined(XARM_GRIPPER)
  xarm_gripper::MoveGoal goal;
  goal.target_pulse = 300;
  goal.pulse_speed = 1500;
  xarm_gripper_.sendGoal(goal);
  bool finishedBeforeTimeout = xarm_gripper_.waitForResult(ros::Duration(3));
  if (finishedBeforeTimeout) {
    ROS_WARN("xarm_gripper_ grasp action did not complete");
    xarm_gripper_.cancelAllGoals();
  }
#else
  control_msgs::GripperCommandGoal goal;
  goal.command.position = 0.5;
  goal.command.max_effort = 10;
  gripper_.sendGoal(goal);
  bool finishedBeforeTimeout = gripper_.waitForResult(ros::Duration(3));
  if (!finishedBeforeTimeout) {
    ROS_WARN("gripper_ grasp action did not complete");
    return false;
  }
#endif
  ROS_INFO("Grasped");

  return true;
}

bool CGrasp::PostGraspCartesian(E_CTRL_TYPE ctrl_type) {
  ROS_INFO("Moving to PostGrasped pose");

  geometry_msgs::TransformStamped tfs_linktcp_on_fixed_frame;
  geometry_msgs::PoseStamped ps_goal_on_fixed_frame;

  try { // link_eefのロボット座標系を基準とした現座標を取得
    tfs_linktcp_on_fixed_frame = olm.tfBuffer_.lookupTransform(FIXED_FRAME, CAR_CTL_EEF_LINK,
        ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    return false;
  }
  transformTFStampedToPoseStamped(tfs_linktcp_on_fixed_frame, ps_goal_on_fixed_frame);

  // 現在の姿勢のままアプローチ座標に戻る
  //ps_goal_on_fixed_frame.pose.position = aprch.approached_link_eef_pose_.pose.position;
  ps_goal_on_fixed_frame.pose.position.z += POSTGRASP_DISTANCE;
  if (ctrl_type == E_CTRL_TYPE::Position) {
    pub_arm_cartesian_.publish(ps_goal_on_fixed_frame);
  } else if (ctrl_type == E_CTRL_TYPE::Velocity) {
    if (!CartesianVelCtrlOnPosCtrl(aprch.grasp_pose_[aprch.POSTGRASP_POSE])) return false;
  } else {
    ROS_WARN("elegal control type.");
    return false;
  }

  ros::Duration(10).sleep();
  ROS_INFO("Moved to PostGrasped pose");

  std::vector<std::string> start_controller;
  start_controller.push_back("xarm6_traj_controller");
  std::vector<std::string> stop_controller;
  stop_controller.push_back("xarm6_cartesian_motion_controller");
  if (false == SwitchController(node_handle, start_controller, stop_controller)) {
    return false;
  }

  return true;
}

bool CGrasp::PickVelocity() {
    std::vector<std::string> start_controller;
    start_controller.push_back("xarm6_cartesian_motion_controller_velocity");
    std::vector<std::string> stop_controller;
    stop_controller.push_back("xarm6_traj_controller_velocity");

    if (false == SwitchController(node_handle, start_controller, stop_controller)) {
        return false;
    }
    //ros::Duration(2).sleep();

    ROS_INFO("Moving to picking pose");
    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = FIXED_FRAME;

    {
        std::lock_guard<std::mutex> lock(olm.mtx_point_);
        copyPose(olm.target_pose_, target_pose.pose);
    }
    target_pose.pose.position.x += 0.01;
    target_pose.pose.position.z -= 0.06;

    static geometry_msgs::PoseStamped diff[2];
    static geometry_msgs::PoseStamped integral;
    memset(&diff[0], 0, sizeof(geometry_msgs::PoseStamped));
    memset(&diff[1], 0, sizeof(geometry_msgs::PoseStamped));
    memset(&integral, 0, sizeof(geometry_msgs::PoseStamped));

    geometry_msgs::PoseStamped* p_target = &target_pose;
    geometry_msgs::PoseStamped ref;
    geometry_msgs::PoseStamped current_pose;;
    geometry_msgs::TransformStamped approached_ts;

    unsigned int count = 0;
#define APPROACHED (0)
#define GRASPING (1)
#define GRASPED (2)
#define POSTGRASPED (3)
    unsigned int status = APPROACHED;
    bool isGrasped = false;
    ros::Rate rate(50);
    while (ros::ok()) {
        try { // link_tcpの現座標を取得
            approached_ts = olm.tfBuffer_.lookupTransform(FIXED_FRAME, "link_tcp", ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return false;
        }
        transformTFStampedToPoseStamped(approached_ts, current_pose);

        float abs = 0.0f;
        if ((abs = std::abs(p_target->pose.position.z - current_pose.pose.position.z)) < 0.005) {
            switch(status) {
            case APPROACHED:
                if (count > 50) {
                    ROS_INFO("Start Grasp: abs = %f, target_z = %f, current_z = %f",
                            abs, p_target->pose.position.z, current_pose.pose.position.z);
                    control_msgs::GripperCommandGoal goal;
                    goal.command.position = 0.5;
                    goal.command.max_effort = 10;
                    gripper_.sendGoal(goal);

                    status = GRASPING;
                    count = 0;
                }
                break;
            case GRASPING:
                if (count > 50) {
                    ROS_INFO("Grasped: abs = %f, target_z = %f, current_z = %f",
                            abs, p_target->pose.position.z, current_pose.pose.position.z);
                    p_target = &aprch.approached_link_tcp_pose_;
                    status = GRASPED;
                    count = 0;
                }
                break;
            case GRASPED:
                if (count > 100) {
                    ROS_INFO("Post Grasped: abs = %f, target_z = %f, current_z = %f",
                            abs, p_target->pose.position.z, current_pose.pose.position.z);
                    ROS_INFO("break: abs = %f", abs);
                    status = POSTGRASPED;
                    break;
                }
            }

            if (status == POSTGRASPED) {
                break;
            }

            count++;
        } else {
            if (count > 0) count--;
        }

        ref.header = p_target->header;

        const float DELTA_T = 1.0/50.0;
        const float KP = 400;
        const float KP_R = 0;
        const float KI = 10;
        const float KI_R = 0;
        const float KD = 5;
        const float KD_R = 0;

        diff[0].pose.position.x = diff[1].pose.position.x;
        diff[0].pose.position.y = diff[1].pose.position.y;
        diff[0].pose.position.z = diff[1].pose.position.z;
        diff[0].pose.orientation.x = diff[1].pose.orientation.x;
        diff[0].pose.orientation.y = diff[1].pose.orientation.y;
        diff[0].pose.orientation.z = diff[1].pose.orientation.z;
        diff[0].pose.orientation.w = diff[1].pose.orientation.w;
        diff[1].pose.position.x = p_target->pose.position.x - current_pose.pose.position.x;
        diff[1].pose.position.y = p_target->pose.position.y - current_pose.pose.position.y;
        diff[1].pose.position.z = p_target->pose.position.z - current_pose.pose.position.z;
        diff[1].pose.orientation.z = p_target->pose.orientation.x - current_pose.pose.orientation.x;
        diff[1].pose.orientation.y = p_target->pose.orientation.y - current_pose.pose.orientation.y;
        diff[1].pose.orientation.z = p_target->pose.orientation.z - current_pose.pose.orientation.z;
        diff[1].pose.orientation.w = p_target->pose.orientation.w - current_pose.pose.orientation.w;

        // TODO 案1：オイラー角でPID計算の後、Quaternionに変換
        // TODO 案2：Quaternionによる回転の補完
        integral.pose.position.x += (diff[1].pose.position.x + diff[0].pose.position.x) / 2.0 * DELTA_T;
        integral.pose.position.y += (diff[1].pose.position.y + diff[0].pose.position.y) / 2.0 * DELTA_T;
        integral.pose.position.z += (diff[1].pose.position.z + diff[0].pose.position.z) / 2.0 * DELTA_T;
        integral.pose.orientation.x += (diff[1].pose.orientation.x + diff[0].pose.orientation.x) / 2.0 * DELTA_T;
        integral.pose.orientation.y += (diff[1].pose.orientation.y + diff[0].pose.orientation.y) / 2.0 * DELTA_T;
        integral.pose.orientation.z += (diff[1].pose.orientation.z + diff[0].pose.orientation.z) / 2.0 * DELTA_T;
        integral.pose.orientation.w += (diff[1].pose.orientation.w + diff[0].pose.orientation.w) / 2.0 * DELTA_T;

        ref.pose.position.x
            = p_target->pose.position.x
            + KP * diff[1].pose.position.x
            + KI * integral.pose.position.x
            + KD * (diff[1].pose.position.x - diff[0].pose.position.x) / DELTA_T;
        ref.pose.position.y
            = p_target->pose.position.y
            + KP * diff[1].pose.position.y
            + KI * integral.pose.position.y
            + KD * (diff[1].pose.position.y - diff[0].pose.position.y) / DELTA_T;
        ref.pose.position.z
            = p_target->pose.position.z
            + KP * diff[1].pose.position.z
            + KI * integral.pose.position.z
            + KD * (diff[1].pose.position.z - diff[0].pose.position.z) / DELTA_T;
        ref.pose.orientation.x
            = p_target->pose.orientation.x
            + KP_R * diff[1].pose.orientation.x
            + KI_R * integral.pose.orientation.x
            + KD_R * (diff[1].pose.orientation.x - diff[0].pose.orientation.x) / DELTA_T;
        ref.pose.orientation.y
            = p_target->pose.orientation.y
            + KP_R * diff[1].pose.orientation.y
            + KI_R * integral.pose.orientation.y
            + KD_R * (diff[1].pose.orientation.y - diff[0].pose.orientation.y) / DELTA_T;
        ref.pose.orientation.z
            = p_target->pose.orientation.z
            + KP_R * diff[1].pose.orientation.z
            + KI_R * integral.pose.orientation.z
            + KD_R * (diff[1].pose.orientation.z - diff[0].pose.orientation.z) / DELTA_T;
        ref.pose.orientation.w
            = p_target->pose.orientation.w
            + KP_R * diff[1].pose.orientation.w
            + KI_R * integral.pose.orientation.w
            + KD_R * (diff[1].pose.orientation.w - diff[0].pose.orientation.w) / DELTA_T;

        pub_arm_cartesian_vel_.publish(ref);

        rate.sleep();
    }

    ROS_INFO("Moved to picking pose");

    start_controller.clear();
    start_controller.push_back("xarm6_traj_controller_velocity");
    stop_controller.clear();
    stop_controller.push_back("xarm6_cartesian_motion_controller_velocity");

    ros::Duration(6).sleep();

    if (false == SwitchController(node_handle, start_controller, stop_controller)) {
        return false;
    }

    return true;
}

#define CAR_VEL_CTL_LOG
bool CGrasp::CartesianVelCtrlOnPosCtrl(geometry_msgs::PoseStamped target_pose) {
    geometry_msgs::Point velo_point;
    geometry_msgs::Vector3 rol_vec3;
    double tgt_roll, tgt_pitch, tgt_yaw;
    double cur_roll, cur_pitch, cur_yaw;
    double vel_roll, vel_pitch, vel_yaw;
    double nxt_roll, nxt_pitch, nxt_yaw;

    geometry_msgs::PoseStamped current_pose;;
    geometry_msgs::TransformStamped approached_ts;
    try { // target_linkの現座標を取得
        approached_ts = olm.tfBuffer_.lookupTransform(FIXED_FRAME, CAR_CTL_EEF_LINK,
                ros::Time(0), ros::Duration(1.0));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        return false;
    }
    transformTFStampedToPoseStamped(approached_ts, current_pose);

    velo_point.x = (target_pose.pose.position.x - current_pose.pose.position.x)
        / std::abs(target_pose.pose.position.x - current_pose.pose.position.x) * CAR_CTL_VEL_P;
    velo_point.y = (target_pose.pose.position.y - current_pose.pose.position.y)
        / std::abs(target_pose.pose.position.y - current_pose.pose.position.y) * CAR_CTL_VEL_P;
    velo_point.z = (target_pose.pose.position.z - current_pose.pose.position.z)
        / std::abs(target_pose.pose.position.z - current_pose.pose.position.z) * CAR_CTL_VEL_P;

    GetRPY(target_pose.pose.orientation, tgt_roll, tgt_pitch, tgt_yaw);
    GetRPY(current_pose.pose.orientation, cur_roll, cur_pitch, cur_yaw);

    vel_roll  = (tgt_roll  - cur_roll ) / std::abs(tgt_roll  - cur_roll ) * CAR_CTL_VEL_R;
    vel_pitch = (tgt_pitch - cur_pitch) / std::abs(tgt_pitch - cur_pitch) * CAR_CTL_VEL_R;
    vel_yaw   = (tgt_yaw   - cur_yaw  ) / std::abs(tgt_yaw   - cur_yaw  ) * CAR_CTL_VEL_R;

    geometry_msgs::PoseStamped next_pose;
    ros::Time now = ros::Time::now();
    ros::Time prev;
    uint32_t seq = 0;
    ros::Rate rate(CAR_CTL_DURATION);
#if defined(CAR_VEL_CTL_LOG)
    unsigned int cnt = 0;
#endif
    while (ros::ok()) {
        uint32_t finished = 0;
        float diff;
        float delta_p;
        double delta_r;
        prev = now;
        now = ros::Time::now();
        ros::Duration duration = now - prev;

        try { // target_linkの現座標を取得
            approached_ts = olm.tfBuffer_.lookupTransform(FIXED_FRAME, CAR_CTL_EEF_LINK,
                    ros::Time(0), ros::Duration(1.0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return false;
        }
        transformTFStampedToPoseStamped(approached_ts, current_pose);

        next_pose.header.seq = seq++;
        next_pose.header.stamp = now;
        next_pose.header.frame_id = FIXED_FRAME;

        vel_roll  = std::abs(tgt_roll  - cur_roll ) > M_PI ? -vel_roll  : vel_roll;
        vel_pitch = std::abs(tgt_pitch - cur_pitch) > M_PI ? -vel_pitch : vel_pitch;
        vel_yaw   = std::abs(tgt_yaw   - cur_yaw  ) > M_PI ? -vel_yaw   : vel_yaw;

        delta_p = velo_point.x * duration.toSec();
        if (std::abs(target_pose.pose.position.x - current_pose.pose.position.x) > std::abs(delta_p)) {
            next_pose.pose.position.x = current_pose.pose.position.x + delta_p;
        } else {
            next_pose.pose.position.x = target_pose.pose.position.x;
            //finished++;
            finished += 1;
        }
        delta_p = velo_point.y * duration.toSec();
        if (std::abs(target_pose.pose.position.y - current_pose.pose.position.y) > std::abs(delta_p)) {
            next_pose.pose.position.y = current_pose.pose.position.y + delta_p;
        } else {
            next_pose.pose.position.y = target_pose.pose.position.y;
            //finished++;
            finished += 10;
        }
        delta_p = velo_point.z * duration.toSec();
        if (std::abs(target_pose.pose.position.z - current_pose.pose.position.z) > std::abs(delta_p)) {
            next_pose.pose.position.z = current_pose.pose.position.z + delta_p;
        } else {
            next_pose.pose.position.z = target_pose.pose.position.z;
            //finished++;
            finished += 100;
        }

        GetRPY(target_pose.pose.orientation, tgt_roll, tgt_pitch, tgt_yaw);
        GetRPY(current_pose.pose.orientation, cur_roll, cur_pitch, cur_yaw);

        delta_r = vel_roll * duration.toSec();
        if ((diff = std::abs(tgt_roll - cur_roll)) > M_PI) diff = std::abs((tgt_roll + 2*M_PI) - cur_roll );
        if (diff > std::abs(delta_r)) {
            nxt_roll = cur_roll + delta_r;
            if (nxt_roll >  M_PI) -M_PI + (nxt_roll - M_PI);
            if (nxt_roll < -M_PI)  M_PI + (nxt_roll + M_PI);
        } else {
            nxt_roll = tgt_roll;
            //finished++;
            finished += 1000;
        }
        delta_r = vel_pitch * duration.toSec();
        if ((diff = std::abs(tgt_pitch - cur_pitch)) > M_PI) diff = std::abs((tgt_pitch + 2*M_PI) - cur_pitch );
        if (diff > std::abs(delta_r)) {
            nxt_pitch = cur_pitch + delta_r;
            if (nxt_pitch >  M_PI) -M_PI + (nxt_pitch - M_PI);
            if (nxt_pitch < -M_PI)  M_PI + (nxt_pitch + M_PI);
        } else {
            nxt_pitch = tgt_pitch;
            //finished++;
            finished += 10000;
        }
        delta_r = vel_yaw * duration.toSec();
        if ((diff = std::abs(tgt_yaw - cur_yaw)) > M_PI) diff = std::abs((tgt_yaw + 2*M_PI) - cur_yaw );
        if (diff > std::abs(delta_r)) {
            nxt_yaw = cur_yaw + delta_r;
            if (nxt_yaw >  M_PI) -M_PI + (nxt_yaw - M_PI);
            if (nxt_yaw < -M_PI)  M_PI + (nxt_yaw + M_PI);
        } else {
            nxt_yaw = tgt_yaw;
            //finished++;
            finished += 100000;
        }

        GetQuaternionMsg(nxt_roll, nxt_pitch, nxt_yaw, next_pose.pose.orientation);

        pub_arm_cartesian_.publish(next_pose);

#if defined(CAR_VEL_CTL_LOG)
        if (cnt % CAR_CTL_DURATION == 0) {
            std::cout << finished << std::endl;
            std::cout << "duration : " << duration.toSec() << std::endl;

            std::cout << "current  : ";
            std::cout << current_pose.pose.position.x;
            std::cout << ", ";
            std::cout << current_pose.pose.position.y;
            std::cout << ", ";
            std::cout << current_pose.pose.position.z;
            std::cout << ", ";
            std::cout << cur_roll;
            std::cout << ", ";
            std::cout << cur_pitch;
            std::cout << ", ";
            std::cout << cur_yaw;
            std::cout << std::endl;
            std::cout << "delta    : ";
            std::cout << velo_point.x * duration.toSec();
            std::cout << ", ";
            std::cout << velo_point.y * duration.toSec();
            std::cout << ", ";
            std::cout << velo_point.z * duration.toSec();
            std::cout << ", ";
            std::cout << vel_roll * duration.toSec();
            std::cout << ", ";
            std::cout << vel_pitch * duration.toSec();
            std::cout << ", ";
            std::cout << vel_yaw * duration.toSec();
            std::cout << std::endl;
            std::cout << "next     : ";
            std::cout << next_pose.pose.position.x;
            std::cout << ", ";
            std::cout << next_pose.pose.position.y;
            std::cout << ", ";
            std::cout << next_pose.pose.position.z;
            std::cout << ", ";
            std::cout << nxt_roll;
            std::cout << ", ";
            std::cout << nxt_pitch;
            std::cout << ", ";
            std::cout << nxt_yaw;
            std::cout << std::endl;
            std::cout << "target   : ";
            std::cout << target_pose.pose.position.x;
            std::cout << ", ";
            std::cout << target_pose.pose.position.y;
            std::cout << ", ";
            std::cout << target_pose.pose.position.z;
            std::cout << ", ";
            std::cout << tgt_roll;
            std::cout << ", ";
            std::cout << tgt_pitch;
            std::cout << ", ";
            std::cout << tgt_yaw;
            std::cout << std::endl;
        }
        if (finished == 111111) break;
        cnt++;
#endif

        rate.sleep();
    }

    return true;
}

