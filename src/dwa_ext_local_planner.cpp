#include "dwa_ext_local_planner/dwa_ext_local_planner.h"


namespace dwa_ext_local_planner
{	
	DWAExtPlanner::DWAExtPlanner(
		std::string name,
		base_local_planner::LocalPlannerUtil *planner_util):
		planner_util_(planner_util),
		path_costs_(planner_util->getCostmap())
	{
		// Allow us to create a private namespace within the move_base
		// node's namespace to avoid conflicts with dynamic reconfigure
		// servers' names
		ros::NodeHandle private_nh("~/" + name);

		// Load parameter from the parameter server
		private_nh.getParam(
			"/move_base/DWAExtPlannerROS/Traversability/display_trajectories",
			DISPLAY_TRAJECTORIES_);

		// Define a string variable to store the name of the controller
		// frequency parameter
		std::string controller_frequency_param_name;
		
		// Get the controller frequency and set the simulation period
		double controller_frequency {};
		private_nh.getParam("/move_base/controller_frequency",
							controller_frequency);
		sim_period_ = 1.0 / controller_frequency;

		ROS_INFO("Sim period is set to %.2f", sim_period_);

		// Load the odometry topic name from the parameter server
      	std::string odom_topic;
		private_nh.getParam(
			"/move_base/DWAExtPlannerROS/Traversability/odom_topic",
			odom_topic);

		// Allow to read the odometry topic
		odom_helper_.setOdomTopic(odom_topic);
		
		// Initialize the oscillation flags
		oscillation_costs_.resetOscillationFlags();

		// Create a vector to store all the different cost functions
    	std::vector<base_local_planner::TrajectoryCostFunction*> critics;

		// Discards oscillating motions (assigns a negative cost)
		// (to prevent the robot from oscillating between some trajectories
		// without making progress the goal, and encourage the robot to explore
		// alternative trajectories)
		critics.push_back(&oscillation_costs_);

		// Prefers trajectories that keep the robot on smooth terrains
		critics.push_back(&traversability_costs_);

		// Prefers trajectories on global path
		critics.push_back(&path_costs_);

		// Create a vector of TrajectorySampleGenerator pointers
		// (generators other than the first are fallback generators)
		std::vector<base_local_planner::TrajectorySampleGenerator*> generators_list;

		// Append the previously created generator to the list
    	generators_list.push_back(&generator_);

		// Initialize the scored sampling planner
    	scored_sampling_planner_ =
			base_local_planner::SimpleScoredSamplingPlanner(generators_list,
															critics);
	}

	DWAExtPlanner::~DWAExtPlanner() {}

	void DWAExtPlanner::reconfigure(DWAExtPlannerConfig &config)
	{	
		// Set the parameters of the trajectory generator
		generator_.setParameters(
        	config.sim_time,
        	config.sim_granularity,
        	config.angular_sim_granularity,
        	config.use_dwa,
        	sim_period_);

		// Set the parameters of the traversability trajectory generator
		generator_traversability_.setParameters(
			4,
			0.05,
			config.angular_sim_granularity,
			config.use_dwa,
			sim_period_);
		
		// Set the parameters of the trajectory generator
      	limits_.max_vel_trans = config.max_vel_trans;
      	limits_.min_vel_trans = config.min_vel_trans;
      	limits_.max_vel_x = config.max_vel_x;
      	limits_.min_vel_x = config.min_vel_x;
      	limits_.max_vel_y = config.max_vel_y;
      	limits_.min_vel_y = config.min_vel_y;
      	limits_.max_vel_theta = config.max_vel_theta;
      	limits_.min_vel_theta = config.min_vel_theta;
      	limits_.acc_lim_x = config.acc_lim_x;
      	limits_.acc_lim_y = config.acc_lim_y;
      	limits_.acc_lim_theta = config.acc_lim_theta;
      	limits_.acc_lim_trans = config.acc_lim_trans;
      	limits_.xy_goal_tolerance = config.xy_goal_tolerance;
      	limits_.yaw_goal_tolerance = config.yaw_goal_tolerance;
      	limits_.prune_plan = config.prune_plan;
      	limits_.trans_stopped_vel = config.trans_stopped_vel;
      	limits_.theta_stopped_vel = config.theta_stopped_vel;

		// Set the parameters of the trajectory generator
      	limits_traversability_.max_vel_trans = 1.0;
      	limits_traversability_.min_vel_trans = 1.0;
      	limits_traversability_.max_vel_x = 0.1;
      	limits_traversability_.min_vel_x = 1.0;
      	limits_traversability_.max_vel_y = 0.0;
      	limits_traversability_.min_vel_y = 0.0;
      	limits_traversability_.max_vel_theta = 0.25;
      	limits_traversability_.min_vel_theta = 0.0;
      	limits_traversability_.acc_lim_x = 3.0;
      	limits_traversability_.acc_lim_y = 100.0;
      	limits_traversability_.acc_lim_theta = 6.0;
      	limits_traversability_.acc_lim_trans = 3.0;
      	limits_traversability_.xy_goal_tolerance = 0.5;
      	limits_traversability_.yaw_goal_tolerance = 0.1;
      	limits_traversability_.prune_plan = config.prune_plan;
      	limits_traversability_.trans_stopped_vel = config.trans_stopped_vel;
      	limits_traversability_.theta_stopped_vel = config.theta_stopped_vel;

		// Reconfigure the planner util
		planner_util_->reconfigureCB(limits_, config.restore_defaults);

		oscillation_costs_.setOscillationResetDist(
			config.oscillation_reset_dist,
			config.oscillation_reset_angle);

		// Store the new configuration
		config_ = config;

		ROS_INFO("Reconfiguration");
	}

	bool DWAExtPlanner::setPlan(
		const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
	{
		// Store the global plan
		planner_util_->setPlan(orig_global_plan);

		// Set the costs for going away from path
    	path_costs_.setTargetPoses(orig_global_plan);

		oscillation_costs_.resetOscillationFlags();

		return true;
	}

	base_local_planner::Trajectory DWAExtPlanner::computeVelocityCommands(
		geometry_msgs::PoseStamped current_pose,
		geometry_msgs::Twist &cmd_vel)
	{
		// Get the current time
		auto time_start { std::chrono::high_resolution_clock::now() };	
		
		// Number of trajectories to sample
		Eigen::Vector3f vsamples(config_.vx_samples,
								 config_.vy_samples,
								 config_.vth_samples);

		// Read the current odometry message on the odometry topic
		// In fact we get the pose and the velocity of the robot expressed
		// in the base_link frame: the velocity is the same that the
		// velocity published on the odometry topic (supposed to be in the
		// frame of the base) and the pose is set to zero
		nav_msgs::Odometry odom;
		odom_helper_.getOdom(odom);
		
		// Get the current pose of the robot in the odometry frame
		Eigen::Vector3f pos(current_pose.pose.position.x,
						    current_pose.pose.position.y,
						    tf2::getYaw(current_pose.pose.orientation));

		// Set the current velocity of the robot
    	Eigen::Vector3f vel(odom.twist.twist.linear.x,
						    odom.twist.twist.linear.y,
						    odom.twist.twist.angular.z);
		
		// Set the goal position
		geometry_msgs::PoseStamped goal_pose;
		planner_util_->getGoal(goal_pose);
		Eigen::Vector3f goal(goal_pose.pose.position.x,
							 goal_pose.pose.position.y,
							 tf2::getYaw(goal_pose.pose.orientation));

		// Initialize the trajectory generator given the current
		// state of the robot
		generator_.initialise(pos, vel, goal, &limits_, vsamples, false);
		
		// Set the current pose of the robot in the robot frame
		Eigen::Vector3f pos_robot_frame { 0.0, 0.0, 0.0 };	

		// Set a smaller number of trajectories on which to predict the
		// traversability cost
		Eigen::Vector3f vsamples_traversability { 1, 1, 3 };

		// Initialize the traversability trajectory generator given the current
		// state of the robot
		generator_traversability_.initialise(pos_robot_frame,
											 vel,
											 goal,
											 &limits_traversability_,
											 vsamples_traversability,
											 false);

		if (DISPLAY_TRAJECTORIES_)
		{
			// Predict the cost of the rectangles
			std::vector<base_local_planner::Trajectory> all_explored_traversability;
			traversability_costs_.predictRectangles(generator_traversability_,
												&all_explored_traversability);

			// Find best trajectory by sampling and scoring the samples
    		// std::vector<base_local_planner::Trajectory> all_explored;
    		// scored_sampling_planner_.findBestTrajectory(result_traj_,
			// 											&all_explored);
    		scored_sampling_planner_.findBestTrajectory(result_traj_, NULL);

			// ROS_INFO("Number of trajectories sampled: %d",
			// 		 static_cast<int>(all_explored.size()));

			traversability_costs_.displayTrajectoriesAndCosts(
				all_explored_traversability);	
		}
		else
		{
			// Predict the cost of the rectangles
			traversability_costs_.predictRectangles(generator_traversability_,
													NULL);
			// Find best trajectory by sampling and scoring the samples
    		scored_sampling_planner_.findBestTrajectory(result_traj_, NULL);
		}

		// Debrief stateful scoring functions
    	oscillation_costs_.updateOscillationFlags(
			pos,
			&result_traj_,
			planner_util_->getCurrentLimits().min_vel_trans);

		// Check if the planner succeeded in finding a valid plan
		if (result_traj_.cost_ < 0.0)
		{
			ROS_WARN("The local planner failed to find a valid plan.\
					 Velocity set to zero.");
			cmd_vel.linear.x = 0.0;
			cmd_vel.linear.y = 0.0;
			cmd_vel.angular.z = 0.0;
		}

		// Fill the velocity command message
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;
		cmd_vel.angular.z = 0.0;
		// cmd_vel.linear.x = result_traj_.xv_;
		// cmd_vel.linear.y = result_traj_.yv_;
		// cmd_vel.angular.z = result_traj_.thetav_;

		// Get the current time
		auto time_stop { std::chrono::high_resolution_clock::now() };

		// Calculate and display the time taken by the planner
		std::chrono::duration<double> time_taken { time_stop - time_start };
		ROS_INFO("Time taken: %f second(s)", time_taken.count());
		ROS_INFO("Frequency: %f Hz \n", 1.0 / time_taken.count());

		return result_traj_;
	}

	bool DWAExtPlanner::checkTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f vel_samples)
	{
    	oscillation_costs_.resetOscillationFlags();

    	base_local_planner::Trajectory traj;
    	geometry_msgs::PoseStamped goal_pose;
		planner_util_->getGoal(goal_pose);

    	Eigen::Vector3f goal(goal_pose.pose.position.x,
							 goal_pose.pose.position.y,
							 tf2::getYaw(goal_pose.pose.orientation));

    	base_local_planner::LocalPlannerLimits limits =
			planner_util_->getCurrentLimits();

		Eigen::Vector3f vsamples(config_.vx_samples,
								 config_.vy_samples,
								 config_.vth_samples);

    	generator_.initialise(pos,
    	    				  vel,
    	    				  goal,
    	    				  &limits,
    	    				  vsamples);

    	generator_.generateTrajectory(pos, vel, vel_samples, traj);

    	double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);

    	// If the trajectory is a legal one... the check passes
    	if(cost >= 0)
    	  return true;
		
    	ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f",
				 vel_samples[0],
				 vel_samples[1],
				 vel_samples[2],
				 cost);

    	// Otherwise the check fails
    	return false;
  	}
}
