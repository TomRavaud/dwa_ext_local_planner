// Include guards to prevent double declarations (eq #pragma once)
#ifndef DWA_EXT_LOCAL_PLANNER_H_
#define DWA_EXT_LOCAL_PLANNER_H_

// Abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

// There are some cool features in goal_functions
#include <base_local_planner/goal_functions.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2/utils.h>

// Dynamic reconfigure
#include <dwa_ext_local_planner/DWAExtPlannerConfig.h>

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

// Custom cost function to assess the terrain traversability
#include "dwa_ext_local_planner/traversability_cost_function.h"

#include <base_local_planner/odometry_helper_ros.h>


namespace dwa_ext_local_planner{
  /**
   * @class DWAExtPlannerROS
   * @brief ROS Wrapper for the DWAExtPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   * 
   */
  class DWAExtPlanner{

    public:

      // /**
      //  * @brief Constructor for DWAExtPlannerROS wrapper
      //  */
      // DWAExtPlanner();

      DWAExtPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util);

      // /**
      //  * @brief Constructor for DWAExtPlannerROS wrapper
      //  */
      // DWAExtPlanner(std::string name, tf2_ros::Buffer* tf,
      //                  costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Destructor for the wrapper
       */
      ~DWAExtPlanner();

      void reconfigure(DWAExtPlannerConfig &config);

      /**
       * @brief Constructs the ROS wrapper
       * 
       * @param name The name to give this instance of the trajectory planner
       * @param tf A pointer to a transform listener
       * @param costmap_ros The cost map to use for assigning costs to
       * trajectories
       */
      void initialize(std::string name, tf2_ros::Buffer* tf,
                      costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Set the plan that the controller is following
       * 
       * @param orig_global_plan The plan to pass to the controller
       * @return True if the plan was updated successfully, false otherwise
       */
      bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

      /**
       * @brief Given the current position, orientation, and velocity of the robot,
       * compute velocity commands to send to the base
       * 
       * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
       * @return True if a valid trajectory was found, false otherwise
       */
      base_local_planner::Trajectory computeVelocityCommands(geometry_msgs::PoseStamped current_pose, geometry_msgs::Twist& cmd_vel);

      /**
       * @brief Check if the goal pose has been achieved
       * 
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

      double getSimPeriod() {return sim_period_;}

      Eigen::Vector3f getAccLimits() {return limits_.getAccLimits();}

      bool checkTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples);

    private:

      dwa_ext_local_planner::DWAExtPlannerConfig config_;

      // Define a trajectory generator
      base_local_planner::SimpleTrajectoryGenerator generator_;

      // Define the traversability cost function
      dwa_ext_local_planner::TraversabilityCostFunction traversability_costs_;

      // Define the global path preference cost function
      base_local_planner::MapGridCostFunction path_costs_;

      // Define the scored sampling planner that will be used to associate the trajectories to costs
      base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;

      // Define a variable to store the best trajectory
      base_local_planner::Trajectory result_traj_;

      // Define a variable to store the current pose of the robot
      // geometry_msgs::PoseStamped current_pose_;

      // Define variables to help us to read the odometry topic
      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_ = "odometry/filtered";

      double sim_period_;

      // Define a variable to store the local planner limits
		  base_local_planner::LocalPlannerLimits limits_;
      
      // Create a local planner util object
      base_local_planner::LocalPlannerUtil *planner_util_;

      ros::Time begin;
  };
};

#endif
