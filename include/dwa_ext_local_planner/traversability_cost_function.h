#ifndef TRAVERSABILITY_COST_FUNCTION_H_
#define TRAVERSABILITY_COST_FUNCTION_H_

#include <base_local_planner/trajectory_cost_function.h>
#include <ros/ros.h>
// #include <Eigen/Core>

// To work with different image transports
#include <image_transport/image_transport.h>
// OpenCV
#include <opencv2/opencv.hpp>
// To convert ROS Images into OpenCV images
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>


namespace dwa_ext_local_planner {
    /**
     * @class TraversabilityCostFunction
     * @brief Uses camera images to extract the robot footprint and assign a
     * cost to trajectories depending on the terrain traversability
     * 
     */
    class TraversabilityCostFunction: public base_local_planner::TrajectoryCostFunction {

        public:

            /**
             * @brief Construct a new Traversability Cost Function object
             * 
             */
            TraversabilityCostFunction();

            /**
             * @brief Destroy the Traversability Cost Function object
             * 
             */
            virtual ~TraversabilityCostFunction();
            
            /**
             * @brief General updating of context values
             * 
             * @return true 
             * @return false 
             */
            virtual bool prepare();

            /**
             * @brief Return a score for trajectory traj
             * 
             * @param traj The trajectory to assign a cost
             * @return double The score of the trajectory
             */
            virtual double scoreTrajectory(base_local_planner::Trajectory &traj);

        private:
            // Define a subscriber to the image topic
            ros::Subscriber sub_image_;

            void callbackImage(const sensor_msgs::ImageConstPtr& image);
    };
}

#endif
