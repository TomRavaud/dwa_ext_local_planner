#ifndef TRAVERSABILITY_COST_FUNCTION_H_
#define TRAVERSABILITY_COST_FUNCTION_H_

// The parent class from which cost functions inherit
#include <base_local_planner/trajectory_cost_function.h>

// ROS C++ API
#include <ros/ros.h>

// To work with different image transports
#include <image_transport/image_transport.h>
// OpenCV
#include <opencv2/opencv.hpp>
// To convert ROS Images into OpenCV images
#include <cv_bridge/cv_bridge.h>

// Some useful ROS messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <time.h>
#include <cstdlib>

// PyTorch C++ API
#include <torch/torch.h>
// TorchScript
#include <torch/script.h>

#include <chrono>
#include <vector>
#include <string>

#include <base_local_planner/simple_trajectory_generator.h>


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
            virtual double scoreTrajectory(
                base_local_planner::Trajectory &traj);

            /**
             * @brief Display the costs of the trajectories
             * 
             * @param trajs The sampled trajectories
             */
            void displayTrajectoriesAndCosts(
                std::vector<base_local_planner::Trajectory> &trajs);

            /**
             * @brief Predict the cost of the rectangles
             * 
             * @param generator The trajectory generator
             * @param all_explored Explored trajectories
             */
            void predictRectangles(
                base_local_planner::SimpleTrajectoryGenerator generator,
                std::vector<base_local_planner::Trajectory>* all_explored=0);

        private:
            /**
             * @brief Callback function to get the camera image
             * 
             * @param image ROS Image message
             */
            void callbackImage(const sensor_msgs::ImageConstPtr& image);
            
            /**
             * @brief Interpolate the traversal cost values on the angular
             * velocity (can be used when only when the robot's linear velocity
             * is fixed)
             * 
             * @param vth The angular velocity of the trajectory
             * to assign a cost
             * @param vth_values The vector of the angular velocities of the
             * trajectories on which the cost has been computed
             * @param cost_values The vector of the computed traversal costs
             * @return double 
             */
            double costInterpolationAngularVel(double vth);

            /**
             * @brief Interpolate the traversal cost of a new trajectory
             * given a set of trajectories on which the cost has been computed
             * 
             * @param x The x coordinate of the trajectory's endpoint
             * @param y The y coordinate of the trajectory's endpoint
             * @return double 
             */
            double costInterpolation(double x, double y);

            // To convert ROS Image type into a CvImage
    	    cv_bridge::CvImagePtr cv_ptr_;

            // Define a subscriber to the image topic
            ros::Subscriber sub_image_;

            // Width of the robot
            double L_;

            // Image width and height
            double IMAGE_W_, IMAGE_H_;

            // Set the tilt angle of the camera
            float alpha_;

            // Define homogeneous transformation matrices
            cv::Mat_<double> robot_to_cam_translation_,
                             cam_to_robot_translation_;
            cv::Mat_<double> robot_to_cam_rotation_,
                             cam_to_robot_rotation_;
            
            // Define an internal calibration matrix
            cv::Mat_<double> K_;

            // Device to run the model on
            torch::Device device_;

            // NN model
            torch::jit::script::Module model_;

            // Define a transform to normalize the image (the first 3 values
            // are the mean and the last 3 are the standard deviation of the
            // dataset ; they were pre-computed on the training set)
            torch::data::transforms::Normalize<> normalize_transform_ =
                torch::data::transforms::Normalize<>({0.4710, 0.5030, 0.4580},
                                                     {0.1965, 0.1859, 0.1955});
                // torch::data::transforms::Normalize<>({0.3426, 0.3569, 0.2914},
                //                                      {0.1363, 0.1248, 0.1302});

            // Set the bins midpoints
            at::Tensor bins_midpoints_;

            // Define vectors to store angular velocities and traversal
            // cost values
            std::vector<double> vth_values_;
            std::vector<double> cost_values_;

            // Define vectors to store the coordinates of the
            // trajectories' endpoints
            std::vector<double> x_values_, y_values_;

            // Distance the robot travels within a patch
            double PATCH_DISTANCE_;  // [m]

            // Ratio between the width and the height of a rectangle
            double RECTANGLE_RATIO_;

            // Maximum number of rectangles to be detected in an image
            int NB_RECTANGLES_MAX_;

            // Choose whether to display the absolute cost color or not
            bool DISPLAY_ABSOLUTE_COST_;
            double COST_MAX_, COST_MIN_;

            // How to interpolate the traversal cost values on the angular
            // velocity
            std::string INTERPOLATION_METHOD_;
            double VTH_THR_;

            // Radius of the circle in which to look for the neighboring
            // trajectories
            double R_;
    };
}

#endif
