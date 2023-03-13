#include "dwa_ext_local_planner/dwa_ext_local_planner.h"

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

	void DWAExtPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
	{
		begin = ros::Time::now(); 

		// Check if the plugin has been initialized
		if (!initialized_)
		{
			// Copy address of costmap and Transform Listener
			// (handed over from move_base)
			costmap_ros_ = costmap_ros;
			tf_ = tf;

			// Set initialized flag
			initialized_ = true;

			ROS_DEBUG("Simple Local Planner plugin initialized.");
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

		// Fill the velocity command message
		cmd_vel.linear.x = 0.5;
		cmd_vel.linear.y = 0;
		cmd_vel.angular.z = 0;

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
