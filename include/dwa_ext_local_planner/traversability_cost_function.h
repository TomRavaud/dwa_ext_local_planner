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

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <time.h>
#include <cstdlib>

#include <torch/torch.h>
#include <torch/script.h>

#include <chrono>


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

            /**
             * @brief Display the costs of the trajectories
             * 
             * @param trajs The sampled trajectories
             */
            void displayCosts(std::vector<base_local_planner::Trajectory> &trajs);

        private:
            void callbackImage(const sensor_msgs::ImageConstPtr& image);

            // To convert ROS Image type into a CvImage
    	    cv_bridge::CvImagePtr cv_ptr_;

            // Define a subscriber to the image topic
            ros::Subscriber sub_image_;

            // Width of the robot
            double L_ = 0.67;

            // Image width and height
            const double IMAGE_W_ = 1280, IMAGE_H_ = 720;

            // Set the tilt angle of the camera
            float alpha_ = -0.197;

            // Define homogeneous transformation matrices
            cv::Mat_<double> robot_to_cam_translation_, cam_to_robot_translation_, world_to_robot_translation_, robot_to_world_translation_;
            cv::Mat_<double> robot_to_cam_rotation_, cam_to_robot_rotation_, world_to_robot_rotation_, robot_to_world_rotation_;

            cv::Mat_<double> cam_to_world_translation_, cam_to_world_rotation_;
            
            // Define an internal calibration matrix
            cv::Mat_<double> K_;

            // Device to run the model on
            torch::Device device_;

            // NN model
            torch::jit::script::Module model_;

            // Define a transform to normalize the image
            torch::data::transforms::Normalize<> normalize_transform_ = torch::data::transforms::Normalize<>({0.3426, 0.3569, 0.2914}, {0.1363, 0.1248, 0.1302});

            // Define the bins midpoints
            at::Tensor bins_midpoints_ = torch::tensor({{0.43156512}, {0.98983318}, {1.19973744}, {1.35943443}, {1.51740755}, {1.67225206}, {1.80821536}, {1.94262708}, {2.12798895}, {2.6080252}}, torch::kFloat);
    };
}

#endif
