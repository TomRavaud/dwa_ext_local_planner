#include "dwa_ext_local_planner/traversability_cost_function.h"


namespace dwa_ext_local_planner {

	void TraversabilityCostFunction::callbackImage(const sensor_msgs::ImageConstPtr& image)
	{   
		// Convert and copy the ROS Image into a CvImage
    	cv_ptr_ = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
	}

    TraversabilityCostFunction::TraversabilityCostFunction() : device_(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU)
    {   
		ros::NodeHandle nh;

        // Initialize the random number generator
        std::srand((unsigned)time(NULL));

        // Initialize the subscriber to the camera image topic
		sub_image_ = nh.subscribe("camera1/image_raw", 1, &dwa_ext_local_planner::TraversabilityCostFunction::callbackImage, this);

        // Set the translation vector
        robot_to_cam_translation_ = (cv::Mat_<double>(3, 1) << 0.084,
                                                               0.060,
                                                               0.774);

        // Set the rotation matrix to rotate the robot frame to the camera frame
        robot_to_cam_rotation_ = (cv::Mat_<double>(3, 3) << 0.0, sin(alpha_), cos(alpha_),
                                                            -1.0, 0.0, 0.0,
                                                            0.0, -cos(alpha_), sin(alpha_));

        // Set the rotation matrix to rotate the camera frame to the robot frame
        cv::invert(robot_to_cam_rotation_, cam_to_robot_rotation_);

        // Set the translation vector to translate the robot frame to the camera frame
        cam_to_robot_translation_ = -cam_to_robot_rotation_*robot_to_cam_translation_;

        // Set the rotation matrix to rotate the world frame to the robot frame
        world_to_robot_rotation_ = cv::Mat::eye(3, 3, CV_32F);

        // Set the rotation matrix to rotate the robot frame to the world frame
        robot_to_world_rotation_ = cv::Mat::eye(3, 3, CV_32F);

        // Set the translation vector to translate the world frame to the robot frame
        world_to_robot_translation_ = cv::Mat::zeros(3, 1, CV_32F);

        // Set the translation vector to translate the robot frame to the world frame
        robot_to_world_translation_ = cv::Mat::zeros(3, 1, CV_32F);
        
        // Set the internal calibration matrix
        K_ = (cv::Mat_<double>(3, 3) << 534, 0, IMAGE_W_/2,
                                        0, 534, IMAGE_H_/2,
                                        0, 0, 1);

        // Compute the homogeneous transformation matrix to transform points from the camera frame to the world frame
        cam_to_world_rotation_ = cam_to_robot_rotation_ * robot_to_world_rotation_;
        cam_to_world_translation_ = cam_to_robot_rotation_ * robot_to_world_translation_ + cam_to_robot_translation_;

        // Device
        std::cout << "Device: " << device_ << std::endl;

        // Load the model
        model_ = torch::jit::load("/home/tom/Traversability-Tom/Husky/src/dwa_ext_local_planner/src/resnet18_classification.pt");

        // Send the model to the GPU
        model_.to(device_);

        // Set the model to inference mode
        model_.eval();

        // Create a vector of inputs
        std::vector<torch::jit::IValue> inputs;

        // Load the test image
        cv::Mat image = cv::imread("/home/tom/Traversability-Tom/Husky/src/dwa_ext_local_planner/src/00000.png");

        // Resize the image
        cv::Mat image_float;
        cv::resize(image, image_float, cv::Size(210, 70));

        // Convert the image to float
        image_float.convertTo(image_float, CV_32FC3, 1.0f / 255.0f);

        // Convert the image to tensor
        at::Tensor tensor = torch::from_blob(image_float.data, {1, image_float.rows, image_float.cols, image_float.channels()}, at::kFloat);
        tensor = tensor.permute({0, 3, 1, 2});

        // Normalize the image
        tensor = normalize_transform_(tensor);

        // Send the tensor to the GPU
        tensor = tensor.to(device_);
        // std::cout << tensor.options() << std::endl;

        // Add the tensor to the input vector
        inputs.push_back(tensor);

        // Execute the model and turn its output into a tensor
        at::Tensor output = model_.forward(inputs).toTensor();
        std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/10) << '\n';
    }

    TraversabilityCostFunction::~TraversabilityCostFunction(){}

    bool TraversabilityCostFunction::prepare() {
        return true;
    }

    double TraversabilityCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj)
    {   
        // Create a vector to store the points of the trajectory in the image frame
        std::vector<cv::Point2d> points_image;

        // Go through the points of the trajectory
        for (int i = 0; i < traj.getPointsSize() - 1; i++)
        {
            // Define variables to store the current pose of the robot in the world frame
            double x;
            double y;
            double th;

            // Get the coordinates of the current point of the trajectory
            traj.getPoint(i, x, y, th);
            
            // Compute the distances between the wheels and the robot's origin
            double delta_X = L_*sin(th)/2;
            double delta_Y = L_*cos(th)/2;

            // Compute the positions of the outer points of the two front wheels
            // (z coordinate is set to 0 because we assume the ground is flat)
            cv::Point3d point_left_world = cv::Point3d(x-delta_X, y+delta_Y, 0.0);
            cv::Point3d point_right_world = cv::Point3d(x+delta_X, y-delta_Y, 0.0);

            // Define vectors to store the coordinates of the current pair of
            // point in the world frame and in the image plan
            std::vector<cv::Point3d> pair_world;
            std::vector<cv::Point2d> pair_image;

            // Add the two point to the vector
            pair_world.push_back(point_left_world);
            pair_world.push_back(point_right_world);

            // Compute the coordinates of those two points in the image plan
            cv::projectPoints(pair_world, cam_to_world_rotation_, cam_to_world_translation_, K_, cv::Mat(), pair_image);

            // Keep only pairs of points contained in the image
            if (pair_image[0].x > 0 && pair_image[0].x < IMAGE_W_ && pair_image[0].y > 0 && pair_image[0].y < IMAGE_H_
                && pair_image[1].x > 0 && pair_image[1].x < IMAGE_W_ && pair_image[1].y > 0 && pair_image[1].y < IMAGE_H_)
            {
                // Add these coordinates to the vector of 2d points
                points_image.push_back(pair_image[0]);
                points_image.push_back(pair_image[1]);
            }
        }

        // Draw circles at the 2D points on the image
        for (int i = 0; i < points_image.size(); i++) {
            cv::circle(cv_ptr_->image, points_image[i], 5, cv::Scalar(0, 0, 255), -1);  // draw a filled circle at the point
        }

        // Display the current image
		cv::imshow("Preview", cv_ptr_->image);
    	cv::waitKey(1);

        // std::cout << traj.thetav_ << std::endl;
        // std::cout << traj.getPointsSize() << std::endl;

        // Compute and return a random cost
        double cost = (double) rand()/RAND_MAX;
        return cost;
    }
}
