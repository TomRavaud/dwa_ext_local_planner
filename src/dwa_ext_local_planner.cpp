#include "dwa_ext_local_planner/dwa_ext_local_planner.h"

// Allow us to register classes as plugins
#include <pluginlib/class_list_macros.h>


//TODO:
#include <vector>
#include <Eigen/Core>

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>

#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/twirling_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>


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

			// Set initialized flag
			initialized_ = true;

			ROS_INFO("Local Planner plugin initialized.");
			
			// Define a dynamic reconfigure server
			config_server_ = new dynamic_reconfigure::Server<DWAExtPlannerConfig>(private_nh);

			// Define a callback function called each time the server gets a
			// reconfiguration request
      		dynamic_reconfigure::Server<DWAExtPlannerConfig>::CallbackType cb;
			cb = boost::bind(
				&dwa_ext_local_planner::DWAExtPlannerROS::callbackReconfigure,
				this, _1, _2);
  			config_server_->setCallback(cb);
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

		// base_local_planner::SimpleTrajectoryGenerator generator_;

		// generator_.setParameters(
        // 	config.sim_time,
        // 	config.sim_granularity,
        // 	config.angular_sim_granularity,
        // 	config.use_dwa,
        // 	sim_period_);

		// std::vector<base_local_planner::TrajectorySampleGenerator*> generator_list;

		// std::cout << generator_list[0] << std::endl;
		// std::cout << generator_list << std::endl;

		// Fill the velocity command message
		// cmd_vel.linear.x = 0.5;
		// cmd_vel.linear.y = 0;
		// cmd_vel.angular.z = 0;

		return true;
	}

	bool DWAExtPlannerROS::isGoalReached()
	{
		// Check if the plugin has been initialized
		if (!initialized_)
		{
			ROS_ERROR("This planner has not been initialized");
			return false;
		}
		
		// Wait for 25 seconds and set the reached goal status to true
		if (ros::Time::now().toSec() - begin.toSec() > 25.0)
		{
			ROS_INFO("Reached goal"); 
			return true; 
		}
		else 
			ROS_INFO_THROTTLE(5, "It takes time to reach the goal"); 

		return false;
	}
}
