// Include guards to prevent double declarations (eq #pragma once)
#ifndef DWA_EXT_LOCAL_PLANNER_ROS_H_
#define DWA_EXT_LOCAL_PLANNER_ROS_H_

// Abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

// ROS C++ API
#include <ros/ros.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dwa_ext_local_planner/DWAExtPlannerConfig.h>

#include <vector>

// A controller to stop the robot when it is close to the goal
// (and correct a deviation on the angular position)
#include <base_local_planner/latched_stop_rotate_controller.h>

// DWA Ext planner class
#include "dwa_ext_local_planner/dwa_ext_local_planner.h"


namespace dwa_ext_local_planner{
  /**
   * @class DWAExtPlannerROS
   * @brief ROS Wrapper for the DWAExtPlanner that adheres to the
   * BaseLocalPlanner interface and can be used as a plugin for move_base.
   * 
   */
  class DWAExtPlannerROS : public nav_core::BaseLocalPlanner{

    public:

      /**
       * @brief Constructor for DWAExtPlannerROS wrapper
       */
      DWAExtPlannerROS();

      /**
       * @brief Constructor for DWAExtPlannerROS wrapper
       */
      DWAExtPlannerROS(std::string name, tf2_ros::Buffer* tf,
                       costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Destructor for the wrapper
       */
      ~DWAExtPlannerROS();

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
      bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

      /**
       * @brief Check if the goal pose has been achieved
       * 
       * @return True if achieved, false otherwise
       */
      bool isGoalReached();

    private:
      /**
       * @brief Callback function called each time the dynamic_reconfigure
       * server is sent a new configuration
       * 
       * @param config The new configuration
       * @param level A bit mask
       */
      void callbackReconfigure(DWAExtPlannerConfig &config, uint32_t level);

      ros::Time begin;

      // The trajectory controller
      boost::shared_ptr<DWAExtPlanner> dwa_ext_planner_;

      // Local cost map attribute
      costmap_2d::Costmap2DROS* costmap_ros_;

      // Pointer to the transform listener
      tf2_ros::Buffer* tf_;

      // To know if the local planner has been initialized or not
      bool initialized_;

      // Pointer to the dynamic reconfigure server
      dynamic_reconfigure::Server<DWAExtPlannerConfig> *config_server_;

      // Create a publisher to publish the global plan
      ros::Publisher global_plan_pub_;

      // Create a publisher to publish the local plan
      ros::Publisher local_plan_pub_;

			base_local_planner::LocalPlannerUtil planner_util_;

      // Create a latch stop rotate controller object
      base_local_planner::LatchedStopRotateController latched_stop_rotate_controller_;

      // Define variables to help us to read the odometry topic
      base_local_planner::OdometryHelperRos odom_helper_;
      std::string odom_topic_ { "odometry/filtered" };

      // Define a variable to store the current pose of the robot
      geometry_msgs::PoseStamped current_pose_;
  };
};

#endif
