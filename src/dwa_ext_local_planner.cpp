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

	void DWAExtPlannerROS::callbackReconfigure(DWAExtPlannerConfig &config, uint32_t level)
	{	
		config_ = config;
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

			// Assuming this planner is being run within the navigation stack, we can
    		// just do an upward search for the frequency at which its being run. This
    		// also allows the frequency to be overwritten locally.
    		std::string controller_frequency_param_name;
    		if(!private_nh.searchParam("controller_frequency", controller_frequency_param_name)) {
    		  sim_period_ = 0.05;
    		} else {
    		  double controller_frequency = 0;
    		  private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
    		  if(controller_frequency > 0) {
    		    sim_period_ = 1.0 / controller_frequency;
    		  } else {
    		    ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
    		    sim_period_ = 0.05;
    		  }
    		}
    		ROS_INFO("Sim period is set to %.2f", sim_period_);

			// Allow to read the odometry topic
			if(private_nh.getParam("odom_topic", odom_topic_))
      		{
      		  odom_helper_.setOdomTopic(odom_topic_);
      		}
			
			// Define a dynamic reconfigure server
			config_server_ = new dynamic_reconfigure::Server<DWAExtPlannerConfig>(private_nh);

			// Define a callback function called each time the server gets a
			// reconfiguration request
      		dynamic_reconfigure::Server<DWAExtPlannerConfig>::CallbackType callback_reconfigure;
			callback_reconfigure = boost::bind(
				&dwa_ext_local_planner::DWAExtPlannerROS::callbackReconfigure,
				this, _1, _2);
  			config_server_->setCallback(callback_reconfigure);

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

		// Set the parameters of the trajectory generator
		generator_.setParameters(
        	config_.sim_time,
        	config_.sim_granularity,
        	config_.angular_sim_granularity,
        	config_.use_dwa,
        	sim_period_);
		
		// Number of trajectories to sample
		Eigen::Vector3f vsamples(config_.vx_samples, config_.vy_samples, config_.vth_samples);
		// Eigen::Vector3f vsamples(10, 1, 10);

		// Get the robot pose in the map
		costmap_ros_->getRobotPose(current_pose_);
		// std::cout << current_pose_.pose.position.x << std::endl;

		// Read the current velocity of the robot on the odometry topic
		geometry_msgs::PoseStamped robot_vel;
    	odom_helper_.getRobotVel(robot_vel);
		// std::cout << robot_vel.pose.position.x << std::endl;

		// std::vector<geometry_msgs::PoseStamped> local_plan;
		// planner_util_.getLocalPlan(current_pose_, local_plan);

		// Set the pos, vel, goal, and limits variables
		Eigen::Vector3f pos(current_pose_.pose.position.x, current_pose_.pose.position.y, tf2::getYaw(current_pose_.pose.orientation));
		// Eigen::Vector3f pos(0.5, 0.4, 0.3);  // set the initial position to (0, 0, 0)
    	Eigen::Vector3f vel(robot_vel.pose.position.x, robot_vel.pose.position.y, tf2::getYaw(robot_vel.pose.orientation));
		// Eigen::Vector3f vel(0, 0, 0);  // set the initial velocity to zero

    	// geometry_msgs::PoseStamped goal_pose = local_plan.back();
    	// Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
		Eigen::Vector3f goal(4.0, 3.0, 5.0);  // set the goal position to (1, 1, 0)

		base_local_planner::LocalPlannerLimits limits;
      	limits.max_vel_trans = config_.max_vel_trans;
      	limits.min_vel_trans = config_.min_vel_trans;
      	limits.max_vel_x = config_.max_vel_x;
      	limits.min_vel_x = config_.min_vel_x;
      	limits.max_vel_y = config_.max_vel_y;
      	limits.min_vel_y = config_.min_vel_y;
      	limits.max_vel_theta = config_.max_vel_theta;
      	limits.min_vel_theta = config_.min_vel_theta;
      	limits.acc_lim_x = config_.acc_lim_x;
      	limits.acc_lim_y = config_.acc_lim_y;
      	limits.acc_lim_theta = config_.acc_lim_theta;
      	limits.acc_lim_trans = config_.acc_lim_trans;
      	limits.xy_goal_tolerance = config_.xy_goal_tolerance;
      	limits.yaw_goal_tolerance = config_.yaw_goal_tolerance;
      	limits.prune_plan = config_.prune_plan;
      	limits.trans_stopped_vel = config_.trans_stopped_vel;
      	limits.theta_stopped_vel = config_.theta_stopped_vel;

		// Initialize the trajectory generator given the current state of the robot
		generator_.initialise(pos, vel, goal, &limits, vsamples, false);

		// base_local_planner::Trajectory traj;
		// std::cout << generator_.nextTrajectory(traj) << std::endl;
		// if (generator_.nextTrajectory(traj))
		// {	
		// 	std::cout << traj.getPointsSize() << std::endl;
		// }

		// Create a vector to store all the different cost functions
    	std::vector<base_local_planner::TrajectoryCostFunction*> critics;
		// Set up the terrain traversability cost function
		critics.push_back(&traversability_costs_);  // Prefers trajectories that keep the robot on smooth terrains

		// Create a vector of TrajectorySampleGenerator pointers
		// (generators other than the first are fallback generators)
		std::vector<base_local_planner::TrajectorySampleGenerator*> generators_list;

		// Append the previously created generator to the list
    	generators_list.push_back(&generator_);

		// Initialize the scored sampling planner
    	scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generators_list, critics);

		// Find best trajectory by sampling and scoring the samples
    	std::vector<base_local_planner::Trajectory> all_explored;
    	scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);

		// Print the cost associated with the best trajectory
		// std::cout << result_traj_.cost_ << std::endl;

		// Fill the velocity command message
		// cmd_vel.linear.x = result_traj_.xv_;
		// cmd_vel.linear.y = result_traj_.yv_;
		// cmd_vel.angular.z = result_traj_.thetav_;

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
		
		// Wait for 200 seconds and set the reached goal status to true
		if (ros::Time::now().toSec() - begin.toSec() > 200.0)
		{
			ROS_INFO("Reached goal"); 
			return true; 
		}
		else 
			ROS_INFO_THROTTLE(5, "It takes time to reach the goal"); 

		return false;
	}
}
