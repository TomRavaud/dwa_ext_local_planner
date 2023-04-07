#include "dwa_ext_local_planner/dwa_ext_local_planner.h"


namespace dwa_ext_local_planner
{	
	DWAExtPlanner::DWAExtPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util) :
		planner_util_(planner_util),
		path_costs_(planner_util->getCostmap())
	{
		// Allow us to create a private namespace within the move_base
		// node's namespace to avoid conflicts with dynamic reconfigure
		// servers' names
		ros::NodeHandle private_nh("~/" + name);

		// Define a string variable to store the name of the controller frequency parameter
		std::string controller_frequency_param_name;

		// If the parameter does not exist, use a default value
		if (!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
			sim_period_ = 0.05;
		else
		{
			double controller_frequency = 0;
			private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);

			if (controller_frequency > 0)
			sim_period_ = 1.0 / controller_frequency;
			else
			{
			ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
			sim_period_ = 0.05;
			}
		}
		ROS_INFO("Sim period is set to %.2f", sim_period_);

		// Allow to read the odometry topic
		odom_helper_.setOdomTopic(odom_topic_);
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

		planner_util_->reconfigureCB(limits_, config.restore_defaults);

		// Store the new configuration
		config_ = config;

		ROS_INFO("Reconfiguration");
	}

	bool DWAExtPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
	{
		// Store the global plan
		planner_util_->setPlan(orig_global_plan);

		// Set the costs for going away from path
    	path_costs_.setTargetPoses(orig_global_plan);

		return true;
	}

	base_local_planner::Trajectory DWAExtPlanner::computeVelocityCommands(geometry_msgs::PoseStamped current_pose, geometry_msgs::Twist &cmd_vel)
	{
		// Get the local plan
		// geometry_msgs::PoseStamped global_pose;
		// planner_util_.getGoal(global_pose);
		// std::vector<geometry_msgs::PoseStamped> plan;
		// planner_util_.getLocalPlan(global_pose, plan);
		// path_costs.setTargetPoses(plan);

		// Get the current time
		auto time_start = std::chrono::high_resolution_clock::now();	
		
		// Number of trajectories to sample
		Eigen::Vector3f vsamples(config_.vx_samples, config_.vy_samples, config_.vth_samples);

		// Read the current odometry message on the odometry topic
		// In fact we get the pose and the velocity of the robot expressed
		// in the base_link frame: the velocity is the same that the
		// velocity published on the odometry topic (supposed to be in the
		// frame of the base) and the pose is set to zero
		nav_msgs::Odometry odom;
		odom_helper_.getOdom(odom);

		// Set the current pose of the robot
		// Eigen::Vector3f pos(odom.pose.pose.position.x,
		// 					odom.pose.pose.position.y,
		// 					tf2::getYaw(odom.pose.pose.orientation));	

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

		// Initialize the trajectory generator given the current state of the robot
		generator_.initialise(pos, vel, goal, &limits_, vsamples, false);

		// Predict the cost of the rectangles
		// traversability_costs_.predictRectangles(generator_);

		// Create a vector to store all the different cost functions
    	std::vector<base_local_planner::TrajectoryCostFunction*> critics;
		// Set up the terrain traversability cost function
		// critics.push_back(&traversability_costs_);  // Prefers trajectories that keep the robot on smooth terrains
		critics.push_back(&path_costs_); // prefers trajectories on global path

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

		ROS_INFO("Number of trajectories sampled: %d", static_cast<int>(all_explored.size()));

		// traversability_costs_.displayTrajectoriesAndCosts(all_explored);

		// Print the cost associated with the best trajectory
		// std::cout << result_traj_.xv_ << '\n';

		// Check if the planner succeeded in finding a valid plan
		if (result_traj_.cost_ < 0.0)
		{
			ROS_WARN("The local planner failed to find a valid plan. Velocity set to zero.");
			cmd_vel.linear.x = 0.0;
			cmd_vel.linear.y = 0.0;
			cmd_vel.angular.z = 0.0;
		}

		// Fill the velocity command message
		cmd_vel.linear.x = result_traj_.xv_;
		cmd_vel.linear.y = result_traj_.yv_;
		cmd_vel.angular.z = result_traj_.thetav_;

		// Get the current time
		auto time_stop = std::chrono::high_resolution_clock::now();

		// Calculate and display the time taken by the planner
		std::chrono::duration<double> time_taken = time_stop - time_start;
		ROS_INFO("Time taken: %f second(s)", time_taken.count());
		ROS_INFO("Frequency: %f Hz \n", 1.0 / time_taken.count());

		return result_traj_;
	}

	bool DWAExtPlanner::checkTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f vel_samples)
	{
    	// oscillation_costs_.resetOscillationFlags();
    	base_local_planner::Trajectory traj;
    	geometry_msgs::PoseStamped goal_pose;
		planner_util_->getGoal(goal_pose);

    	Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));

    	base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();

		Eigen::Vector3f vsamples(config_.vx_samples, config_.vy_samples, config_.vth_samples);

    	generator_.initialise(pos,
    	    vel,
    	    goal,
    	    &limits,
    	    vsamples);

    	generator_.generateTrajectory(pos, vel, vel_samples, traj);
    	double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
    	//if the trajectory is a legal one... the check passes
    	if(cost >= 0) {
    	  return true;
    	}
    	ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

    	//otherwise the check fails
    	return false;
  	}
}
