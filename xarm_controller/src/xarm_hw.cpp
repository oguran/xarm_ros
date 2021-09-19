/* Copyright 2018 UFACTORY Inc. All Rights Reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Author: Jason Peng <jason@ufactory.cc>
 ============================================================================*/

#include "xarm_controller/xarm_hw.h"

#define SERVICE_IS_PERSISTENT_BUT_INVALID 998

namespace xarm_control
{
	void XArmHW::_register_joint_limits(ros::NodeHandle &root_nh, std::string joint_name, const ControlMethod ctrl_method)
	{
		joint_limits_interface::JointLimits joint_limits;
		joint_limits_interface::SoftJointLimits soft_joint_limits;
		bool has_limits = false;
		bool has_soft_limits = false;
		
		urdf::JointConstSharedPtr urdf_joint = model_ptr_->getJoint(joint_name);
		if (urdf_joint != NULL) 
		{
			// Get limits from the URDF file.
			if (joint_limits_interface::getJointLimits(urdf_joint, joint_limits))
				has_limits = true;
			if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_joint_limits))
				has_soft_limits = true;
		}
		// Get limits from the parameter server.
		if (joint_limits_interface::getJointLimits(joint_name, root_nh, joint_limits))
			has_limits = true;

		if (!has_limits)
			return;

		switch (ctrl_method)
		{
		case EFFORT:
			{
				if (has_soft_limits)
					ej_limits_interface_.registerHandle(joint_limits_interface::EffortJointSoftLimitsHandle(ej_interface_.getHandle(joint_name), joint_limits, soft_joint_limits));
				else
					ej_sat_interface_.registerHandle(joint_limits_interface::EffortJointSaturationHandle(ej_interface_.getHandle(joint_name), joint_limits));
			}
			break;
		case VELOCITY:
			{
				if (has_soft_limits)
					vj_limits_interface_.registerHandle(joint_limits_interface::VelocityJointSoftLimitsHandle(vj_interface_.getHandle(joint_name), joint_limits, soft_joint_limits));
				else
					vj_sat_interface_.registerHandle(joint_limits_interface::VelocityJointSaturationHandle(vj_interface_.getHandle(joint_name), joint_limits));
			}
			break;
		case POSITION:
		default:
			{
				if (has_soft_limits)
					pj_limits_interface_.registerHandle(joint_limits_interface::PositionJointSoftLimitsHandle(pj_interface_.getHandle(joint_name), joint_limits, soft_joint_limits));
				else
					pj_sat_interface_.registerHandle(joint_limits_interface::PositionJointSaturationHandle(pj_interface_.getHandle(joint_name), joint_limits));
			}
			break;
		}
		printf("%s, ctrl_method=%d, has_soft_limits=%d, has_velocity_limits=%d, max_velocity=%f, has_position_limits=%d, min_position=%f, max_position=%f\n", 
				joint_name.c_str(), ctrl_method, has_soft_limits, joint_limits.has_velocity_limits, joint_limits.max_velocity, 
				joint_limits.has_position_limits, joint_limits.min_position, joint_limits.max_position);
	}

	void XArmHW::clientInit(const std::string& robot_ip, ros::NodeHandle& root_nh)
	{
		position_cmd_.resize(dof_);
		position_cmd_float_.resize(dof_); // command vector must have 7 dimention!
		position_fdb_.resize(dof_);
		velocity_cmd_.resize(dof_);
		velocity_cmd_float_.resize(dof_);
		velocity_fdb_.resize(dof_);
		effort_cmd_.resize(dof_);
		effort_fdb_.resize(dof_);

		prev_cmds_float_.resize(dof_);

		curr_err = 0;
		curr_state = 0;
		service_fail_ret = 0;

		pos_sub_ = root_nh.subscribe(jnt_state_topic, 100, &XArmHW::pos_fb_cb, this);
		state_sub_ = root_nh.subscribe(xarm_state_topic, 100, &XArmHW::state_fb_cb, this);

		for(unsigned int j=0; j < dof_; j++)
	  	{
			// Create joint state interface for all joints
			js_interface_.registerHandle(hardware_interface::JointStateHandle(jnt_names_[j], &position_fdb_[j], &velocity_fdb_[j], &effort_fdb_[j]));
			switch (ctrl_method_)
			{
			case EFFORT:
				{
					ej_interface_.registerHandle(hardware_interface::JointHandle(js_interface_.getHandle(jnt_names_[j]), &effort_cmd_[j]));
				}
				break;
			case VELOCITY:
				{
					vj_interface_.registerHandle(hardware_interface::JointHandle(js_interface_.getHandle(jnt_names_[j]), &velocity_cmd_[j]));
				}
				break;
			case POSITION:
			default:
				{
					ctrl_method_ = POSITION;
					pj_interface_.registerHandle(hardware_interface::JointHandle(js_interface_.getHandle(jnt_names_[j]), &position_cmd_[j]));
				}
				break;
			}
			_register_joint_limits(root_nh, jnt_names_[j], ctrl_method_);
	  	}

		registerInterface(&js_interface_);
		switch (ctrl_method_)
		{
		case EFFORT:
			registerInterface(&ej_interface_);
			break;
		case VELOCITY:
			registerInterface(&vj_interface_);
			break;
		case POSITION:
		default:
			registerInterface(&pj_interface_);
			break;
		}
	  	
	  	int ret1 = xarm.motionEnable(1);
	  	int ret2 = xarm.setMode(ctrl_method_ == VELOCITY ? XARM_MODE::VELO_JOINT : XARM_MODE::SERVO);
	  	int ret3 = xarm.setState(XARM_STATE::START);

	  	if(ret3)
	  	{
	  		ROS_ERROR("The Xarm may not be properly connected (ret = 3) or hardware Error/Warning (ret = 1 or 2) exists, PLEASE CHECK or RESTART HARDWARE!!!");
	  		ROS_ERROR(" ");
	  		ROS_ERROR("Did you specify the correct ros param xarm_robot_ip ? Exitting...");
	  		ros::shutdown();
	  		exit(1);
	  	}

	}

	bool XArmHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh)
	{
		bool velocity_control = false;
		robot_hw_nh.getParam("velocity_control", velocity_control);
		// ctrl_method_ = EFFORT; // INVALID
		// ctrl_method_ = VELOCITY; // INVALID
		if (velocity_control) {
			ctrl_method_ = VELOCITY;
		}
		else {
			ctrl_method_ = POSITION; // default
		}

		hw_ns_ = robot_hw_nh.getNamespace() + "/";
		ros::service::waitForService(hw_ns_+"motion_ctrl");
	  	ros::service::waitForService(hw_ns_+"set_state");
	  	ros::service::waitForService(hw_ns_+"set_mode");
		if (ctrl_method_ == VELOCITY)
			ros::service::waitForService(hw_ns_+"velo_move_joint");
		else
	  		ros::service::waitForService(hw_ns_+"move_servoj");
		xarm.init(robot_hw_nh);
		std::string robot_ip;
		std::vector<std::string> jnt_names;
		int xarm_dof = 0;

		if(!robot_hw_nh.hasParam("DOF"))
		{
			ROS_ERROR("ROS Parameter xarm_dof not specified!");
			return false;
		}
		if(!robot_hw_nh.hasParam("xarm_robot_ip"))
		{
			ROS_ERROR("ROS Parameter xarm_robot_ip not specified!");
			return false;
		}
		// If there is no /robot_description parameter, moveit controller may send zero command even controller fails to initialize
		if(!robot_hw_nh.hasParam("/robot_description"))
		{
			ROS_ERROR("ROS Parameter /robot_description not specified!");
			return false;
		}

		/* getParam forbids to change member */
		robot_hw_nh.getParam("DOF", xarm_dof);
		robot_hw_nh.getParam("xarm_robot_ip", robot_ip);
		robot_hw_nh.getParam("joint_names", jnt_names);

		dof_ = xarm_dof;
		jnt_names_ = jnt_names;
		initial_write_ = true;

		std::string robot_description;
		root_nh.getParam("/robot_description", robot_description);
		model_ptr_ = urdf::parseURDF(robot_description);

		clientInit(robot_ip, robot_hw_nh);
		return true;
	}

	XArmHW::~XArmHW()
	{
		xarm.setMode(XARM_MODE::POSE);
	}

	void XArmHW::pos_fb_cb(const sensor_msgs::JointState::ConstPtr& data)
	{
		std::lock_guard<std::mutex> locker(mutex_);
		for(int j=0; j<dof_; j++)
		{
			position_fdb_[j] = data->position[j];
			velocity_fdb_[j] = data->velocity[j];
			effort_fdb_[j] = data->effort[j];
		}
	}

	void XArmHW::state_fb_cb(const xarm_msgs::RobotMsg::ConstPtr& data)
	{
		curr_mode = data->mode;
		curr_state = data->state;
		curr_err = data->err;
	}

	void XArmHW::_reset_limits(void)
	{
		pj_sat_interface_.reset();
		pj_limits_interface_.reset();
	}

	void XArmHW::_enforce_limits(const ros::Duration& period)
	{
		switch (ctrl_method_)
		{
		case EFFORT:
			{
				ej_sat_interface_.enforceLimits(period);
				ej_limits_interface_.enforceLimits(period);
			}
			break;
		case VELOCITY:
			{
				vj_sat_interface_.enforceLimits(period);
				vj_limits_interface_.enforceLimits(period);
			}
			break;
		case POSITION:
		default:
			{
				pj_sat_interface_.enforceLimits(period);
				pj_limits_interface_.enforceLimits(period);
			}
			break;
		}
	}

	void XArmHW::read(const ros::Time& time, const ros::Duration& period)
	{
		// basically the above feedback callback functions have done the job
	}

	void XArmHW::write(const ros::Time& time, const ros::Duration& period)
	{
		if(initial_write_ || need_reset())
		{
			std::lock_guard<std::mutex> locker(mutex_);
			for(int k=0; k<dof_; k++)
			{
				position_cmd_float_[k] = (float)position_fdb_[k];
				velocity_cmd_float_[k] = 0;
			}
			_reset_limits();
			initial_write_ = false;
			return;
		}

		_enforce_limits(period);

		int cmd_ret = 0;
		switch (ctrl_method_)
		{
		case VELOCITY:
			{
				for (int k = 0; k < dof_; k++) { velocity_cmd_float_[k] = (float)velocity_cmd_[k]; }
				cmd_ret = xarm.veloMoveJoint(velocity_cmd_float_, true);
			}
			break;
		case POSITION:		
		default:
			{
				for (int k = 0; k < dof_; k++) { position_cmd_float_[k] = (float)position_cmd_[k]; }
				cur_time_ = ros::Time::now();
				elapsed_ = cur_time_ - prev_time_;
				if (elapsed_.toSec() > 1 || _check_cmds_is_change(prev_cmds_float_, position_cmd_float_)) {
					cmd_ret = xarm.setServoJ(position_cmd_float_);
					if (cmd_ret == 0) {
						prev_time_ = cur_time_;
						for (int i = 0; i < prev_cmds_float_.size(); i++) { 
							prev_cmds_float_[i] = (float)position_cmd_float_[i];
						}
					}
				}	
			}
			break;
		}

		if (cmd_ret != 0 && cmd_ret != UXBUS_STATE::WAR_CODE) {
			// to reset controller, preempt current goal
			service_fail_ret = cmd_ret;
		}
		// for(int k=0; k<dof_; k++)
		// {
		// 	// make sure no abnormal command will be written into joints, check if cmd velocity > [180 deg/sec * (1+10%)]
		// 	if(fabs(position_cmd_float_[k]-(float)position_cmd_[k])/(period.toSec()) > 3.14*1.25  && !initial_write_)
		// 	{
		// 		ROS_WARN("joint %d abnormal command! previous: %f, this: %f\n", k+1, position_cmd_float_[k], (float)position_cmd_[k]);
		// 		// return;
		// 	}

		// 	position_cmd_float_[k] = (float)position_cmd_[k];
		// }

		// xarm.setServoJ(position_cmd_float_);
		
		// initial_write_ = false;
	}

	bool XArmHW::_check_cmds_is_change(std::vector<float> prev, std::vector<float> cur, double threshold)
    {
        for (int i = 0; i < cur.size(); i++) {
            if (std::abs(cur[i] - prev[i]) > threshold) return true;
        }
        return false;
    }

	void XArmHW::get_status(int state_mode_err[3])
	{
		state_mode_err[0] = curr_state;
		state_mode_err[1] = curr_mode;
		state_mode_err[2] = curr_err;
	}

	bool XArmHW::need_reset()
	{	
		static int last_err = 0;
		if((ctrl_method_ == VELOCITY ? curr_mode != XARM_MODE::VELO_JOINT : curr_mode != XARM_MODE::SERVO) 
			|| curr_state==4 || curr_state==5 || curr_err || service_fail_ret)
		{
			if(last_err != curr_err && curr_err)
			{
				ROS_ERROR("[ns: %s] xArm Error detected! Code: %d", hw_ns_.c_str(), curr_err);
				last_err = curr_err;
			}
			if (service_fail_ret != 0) {
				int ret = xarm.setState(XARM_STATE::STOP);
				ROS_ERROR("XArmHW::Write() failed, failed_ret=%d !, Setting Robot State to STOP... (ret: %d)", service_fail_ret, ret);
				if (service_fail_ret == SERVICE_IS_PERSISTENT_BUT_INVALID) {
					ROS_ERROR("service is invaild, ros shutdown");
					ros::shutdown();
	  				exit(1);
				}
				service_fail_ret = 0;
			}
			// ROS_ERROR("Need Reset returns true! ctrl_method_: %d, curr_mode: %d, curr_state: %d, curr_err: %d", ctrl_method_, curr_mode, curr_state, curr_err);
			return true;
		}
		else
		{
			last_err = 0;
			return false;
		}
	}
}

PLUGINLIB_EXPORT_CLASS(xarm_control::XArmHW, hardware_interface::RobotHW)