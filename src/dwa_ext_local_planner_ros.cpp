#include "dwa_ext_local_planner/dwa_ext_local_planner_ros.h"

// Allow us to register classes as plugins
#include <pluginlib/class_list_macros.h>


// Register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwa_ext_local_planner::DWAExtPlannerROS,
					   nav_core::BaseLocalPlanner)

namespace dwa_ext_local_planner
{
	DWAExtPlannerROS::DWAExtPlannerROS()
		: costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

	DWAExtPlannerROS::DWAExtPlannerROS(std::string name, tf2_ros::Buffer *tf,
									   costmap_2d::Costmap2DROS *costmap_ros)
		: costmap_ros_(NULL), tf_(NULL), initialized_(false)
	{
		// Initialize the planner
		initialize(name, tf, costmap_ros);
	}

	DWAExtPlannerROS::~DWAExtPlannerROS() {}

	void DWAExtPlannerROS::callbackReconfigure(DWAExtPlannerConfig &config, uint32_t level)
	{	
		// Reconfigure the planner
		dwa_ext_planner_->reconfigure(config);

		ROS_INFO("Reconfiguration");
	}

	void DWAExtPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
	{
		begin = ros::Time::now(); 

		// Check if the plugin has been initialized
		if (!initialized_)
		{
			// Allow us to create a private namespace within the move_base
			// node's namespace to avoid conflicts with dynamic reconfigure
			// servers' names
			ros::NodeHandle private_nh("~/" + name);

			// Copy address of costmap and Transform Listener
			// (handed over from move_base)
			costmap_ros_ = costmap_ros;
			tf_ = tf;
			
			ROS_INFO("Costmap global frame is %s", costmap_ros_->getGlobalFrameID().c_str());
			ROS_INFO("Costmap local frame is %s", costmap_ros_->getBaseFrameID().c_str());
			
			// Initialize the local planner util
			planner_util_.initialize(tf,
									costmap_ros->getCostmap(),
									costmap_ros->getGlobalFrameID());

			// Create the local planner
			dwa_ext_planner_ = boost::shared_ptr<DWAExtPlanner>(new DWAExtPlanner(name, &planner_util_));

			// Define a dynamic reconfigure server
			config_server_ = new dynamic_reconfigure::Server<DWAExtPlannerConfig>(private_nh);

			// Define a callback function called each time the server gets a
			// reconfiguration request
      		dynamic_reconfigure::Server<DWAExtPlannerConfig>::CallbackType callback_reconfigure;
			callback_reconfigure = boost::bind(
				&dwa_ext_local_planner::DWAExtPlannerROS::callbackReconfigure,
				this, _1, _2);
  			config_server_->setCallback(callback_reconfigure);

			// Advertise the global plan publisher
			global_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);

			// Advertise the local plan publisher
			local_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);

			// Allow to read the odometry topic
			odom_helper_.setOdomTopic(odom_topic_);

			// Set initialized flag
			initialized_ = true;

			ROS_INFO("Local Planner plugin initialized.");
		}
		else
		{
			ROS_WARN("This planner has already been initialized, doing nothing.");
		}
	}

	bool DWAExtPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
	{
		// Check if the plugin has been initialized
		if (!initialized_)
		{
			ROS_ERROR("This planner has not been initialized");
			return false;
		}

		// The global planner sends the plan to follow once the goal has been given
		ROS_INFO("Got new plan");

		// When we get a new plan, we also want to clear any latch we may have on goal tolerances
    	// latched_stop_rotate_controller_.resetLatching();

		dwa_ext_planner_->setPlan(orig_global_plan);
		
		// Set the global path preference cost function
		// path_costs_.setTargetPoses(orig_global_plan);

		// Publish the plan for visualization purposes
		base_local_planner::publishPlan(orig_global_plan, global_plan_pub_);

		//when we get a new plan, we also want to clear any latch we may have on goal tolerances
    	latched_stop_rotate_controller_.resetLatching();

		// Get the global pose
		// geometry_msgs::PoseStamped global_pose;
		// planner_util_.getGoal(global_pose);
		// std::cout << "Goal: " << global_pose << std::endl;

		// Get the local plan
		// std::vector<geometry_msgs::PoseStamped> transformed_plan;
		// planner_util_.getLocalPlan(global_pose, transformed_plan);
		// std::cout << "Local plan: " << transformed_plan << std::endl;

		return true;
	}

	bool DWAExtPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
	{
		// Check if the plugin has been initialized
		if (!initialized_)
		{
			ROS_ERROR("This planner has not been initialized");
			return false;
		}
		
		// Get the current pose of the robot
		costmap_ros_->getRobotPose(current_pose_);
		
		if (latched_stop_rotate_controller_.isPositionReached(&planner_util_, current_pose_))
		{
    	  return latched_stop_rotate_controller_.computeVelocityCommandsStopRotate(
    	      cmd_vel,
    	      dwa_ext_planner_->getAccLimits(),
    	      dwa_ext_planner_->getSimPeriod(),
    	      &planner_util_,
    	      odom_helper_,
    	      current_pose_,
    	      [this](auto pos, auto vel, auto vel_samples){ return dwa_ext_planner_->checkTrajectory(pos, vel, vel_samples); });
    	}
		else
		{
			// Compute the velocity command to send to the base
			base_local_planner::Trajectory result_traj = dwa_ext_planner_->computeVelocityCommands(current_pose_, cmd_vel);

			// Create the local plan
			std::vector<geometry_msgs::PoseStamped> local_plan;

			// Fill out the local plan
    		for(unsigned int i = 0; i < result_traj.getPointsSize(); ++i)
			{
    		  double p_x, p_y, p_th;
    		  result_traj.getPoint(i, p_x, p_y, p_th);

    		  geometry_msgs::PoseStamped p;
    		  p.header.frame_id = costmap_ros_->getGlobalFrameID();
    		  p.header.stamp = ros::Time::now();
    		  p.pose.position.x = p_x;
    		  p.pose.position.y = p_y;
    		  p.pose.position.z = 0.0;
    		  tf2::Quaternion q;
    		  q.setRPY(0, 0, p_th);
    		  tf2::convert(q, p.pose.orientation);
    		  local_plan.push_back(p);
    		}

    		// Publish the local plan for visualization purposes
			base_local_planner::publishPlan(local_plan, local_plan_pub_);

			return true;
		}
	}

	bool DWAExtPlannerROS::isGoalReached()
	{
		// Check if the plugin has been initialized
		if (!initialized_)
		{
			ROS_ERROR("This planner has not been initialized");
			return false;
		}
		
		ROS_INFO_THROTTLE(5, "Reaching the goal"); 
		
		// Check if the goal has been reached (position and orientation)
		if (latched_stop_rotate_controller_.isGoalReached(&planner_util_, odom_helper_, current_pose_))
		{
			ROS_INFO("Reached goal"); 
			return true; 
		}

		return false;
	}
}
