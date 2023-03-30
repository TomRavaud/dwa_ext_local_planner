#include "dwa_ext_local_planner/traversability_cost_function.h"


namespace dwa_ext_local_planner {

	void TraversabilityCostFunction::callbackImage(const sensor_msgs::ImageConstPtr& image)
	{   
		// Convert and copy the ROS Image into a CvImage
    	cv_ptr_ = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
	}

    TraversabilityCostFunction::TraversabilityCostFunction() : device_(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU)
    {   
		ros::NodeHandle nh;

        // Initialize the random number generator
        std::srand((unsigned)time(NULL));

        // Initialize the subscriber to the camera image topic
		sub_image_ = nh.subscribe("zed_node/rgb/image_rect_color", 1, &dwa_ext_local_planner::TraversabilityCostFunction::callbackImage, this);
		// sub_image_ = nh.subscribe("camera1/image_raw", 1, &dwa_ext_local_planner::TraversabilityCostFunction::callbackImage, this);

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

        // Set the internal calibration matrix
        K_ = (cv::Mat_<double>(3, 3) << 534, 0, IMAGE_W_/2,
                                        0, 534, IMAGE_H_/2,
                                        0, 0, 1);

        // Device
        std::cout << "Device: " << device_ << std::endl;

        // Load the model
        model_ = torch::jit::load("/home/tom/Traversability-Tom/Husky/src/dwa_ext_local_planner/src/resnet18_classification2.pt");

        // Send the model to the GPU
        model_.to(device_);

        // Send the bins midpoints to the GPU
        bins_midpoints_ = bins_midpoints_.to(device_);
    }

    TraversabilityCostFunction::~TraversabilityCostFunction(){}

    bool TraversabilityCostFunction::prepare()
    {
        // Initialize the index of the trajectory
        index_trajectory_ = 0;

        // Initialize the index of the rectangle
        index_rectangle_ = 0;

        return true;
    }

    double TraversabilityCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj)
    {   
        // Get the number of rectangles in the current trajectory
        int nb_rectangles = nb_rectangles_vector_[index_trajectory_];

        // Disable gradient computation 
        at::NoGradGuard no_grad;

        // Extract the predicted costs on the rectangles of the current trajectory
        at::Tensor predicted_costs_rectangles_trajectory = predicted_costs_rectangles_.narrow(0,
                                                                                              index_rectangle_,
                                                                                              nb_rectangles);

        // Increment the index of the trajectory
        index_trajectory_++;

        // Increment the index of the rectangle
        index_rectangle_ += nb_rectangles;

        // Compute the expected costs over the bins
        at::Tensor expected_costs_rectangles = at::mm(predicted_costs_rectangles_trajectory,
                                                      bins_midpoints_);

        // Get the maximum cost
        double cost = at::max(expected_costs_rectangles).item<double>();

        return cost;
    }

    void TraversabilityCostFunction::displayTrajectoriesAndCosts(std::vector<base_local_planner::Trajectory> &trajs)
    {   
        // Get the current image
        cv::Mat image_to_display = cv_ptr_->image.clone();

        // Display the costs of the trajectories
        for (int i = 0; i < trajs.size(); i++)
        {   
            std::vector<cv::Point2d> previous_pair_image;

            // Create vectors to store the points of the trajectory
            std::vector<cv::Point> points_left;
            std::vector<cv::Point> points_right;
            std::vector<cv::Point> points;

            std::cout << trajs[i].xv_ << std::endl;
            std::cout << trajs[i].yv_ << std::endl;
            std::cout << trajs[i].thetav_ << std::endl;

            for (int j = 0; j < trajs[i].getPointsSize(); j++)
            {
                // Define variables to store the current pose of the robot in the robot frame
                double x, y, th;

                // Get the coordinates of the current point of the trajectory
                trajs[i].getPoint(j, x, y, th);

                // Compute the distances between the wheels and the robot's origin
                double delta_X = L_*sin(th)/2;
                double delta_Y = L_*cos(th)/2;

                // Compute the positions of the outer points of the two front wheels
                // (z coordinate is set to 0 because we assume the ground is flat)
                cv::Point3d current_point_left_robot = cv::Point3d(x-delta_X, y+delta_Y, 0.0);
                cv::Point3d current_point_right_robot = cv::Point3d(x+delta_X, y-delta_Y, 0.0);

                // Define vectors to store the coordinates of the current pair of
                // point in the robot frame and in the image plan
                std::vector<cv::Point3d> current_pair_robot;
                std::vector<cv::Point2d> current_pair_image;

                // Add the two points to the vector
                current_pair_robot.push_back(current_point_left_robot);
                current_pair_robot.push_back(current_point_right_robot);

                // Compute the coordinates of those two points in the image plan
                cv::projectPoints(current_pair_robot, cam_to_robot_rotation_, cam_to_robot_translation_, K_, cv::Mat(), current_pair_image);

                // Keep only pairs of points contained in the image
                if (current_pair_image[0].x > 0 && current_pair_image[0].x < IMAGE_W_ && current_pair_image[0].y > 0 && current_pair_image[0].y < IMAGE_H_
                    && current_pair_image[1].x > 0 && current_pair_image[1].x < IMAGE_W_ && current_pair_image[1].y > 0 && current_pair_image[1].y < IMAGE_H_)
                {   
                    // Draw circles at the 2D points on the image
                    // cv::circle(image_to_display, current_pair_image[0], 5, cv::Scalar(0, 0, 255), -1);  // draw a filled circle at the point
                    // cv::circle(image_to_display, current_pair_image[1], 5, cv::Scalar(0, 0, 255), -1);  // draw a filled circle at the point

                    points_left.push_back(cv::Point(current_pair_image[0].x, current_pair_image[0].y));
                    points_right.push_back(cv::Point(current_pair_image[1].x, current_pair_image[1].y));

                    // If there is no previous pair of points, store the current pair of points
                    if (previous_pair_image.size() == 0)
                    {
                        previous_pair_image.push_back(current_pair_image[0]);
                        previous_pair_image.push_back(current_pair_image[1]);
                        continue;
                    }

                    // Draw the bounding box
                    // cv::rectangle(image_to_display, cv::Point(
                    //                                           std::min(current_pair_image[0].x, previous_pair_image[0].x),
                    //                                           std::min(current_pair_image[0].y, current_pair_image[1].y)
                    //                                           ),
                    //                                 cv::Point(
                    //                                           std::max(current_pair_image[1].x, previous_pair_image[1].x),
                    //                                           std::max(previous_pair_image[0].y, previous_pair_image[1].y)
                    //                                           ),
                    //                                           cv::Scalar(0, 255, 0));

                    // Store the current pair of points as the previous pair of points
                    previous_pair_image.clear();
                    previous_pair_image.push_back(current_pair_image[0]);
                    previous_pair_image.push_back(current_pair_image[1]);
                }
            }
            std::cout << "Trajectory " << i << " cost: " << trajs[i].cost_ << std::endl;

            // Fill the vectors of points with the left and right points
            points.insert(points.end(), points_left.begin(), points_left.end());
            std::reverse(points_right.begin(), points_right.end());
            points.insert(points.end(), points_right.begin(), points_right.end());

            if (points.size() == 0)
                continue;
            
            // Create a vector of vectors of points to be able to use the fillPoly function
            std::vector<std::vector<cv::Point>> points_vector = {points};

            // Create a copy of the image to display
            cv::Mat overlay = image_to_display.clone();

            // Set the maximum and minimum costs
            double cost_max = 2.5, cost_min = 0.0;

            // Compute the green and red values to display the cost
            double green = 255/(cost_min - cost_max)*trajs[i].cost_ + 255*cost_max/(cost_max - cost_min);
            double red = 255 - green;

            // Draw the polygon
            cv::fillPoly(overlay, points_vector, cv::Scalar(0, green, red));

            // Weighted sum of the original image and the overlay to create transparency
            double transparency = 0.3;  // Specify the level of transparency
            cv::addWeighted(overlay, transparency, image_to_display, 1 - transparency, 0, image_to_display);
        }
        // Display the current image
		cv::imshow("Preview", image_to_display);
    	cv::waitKey(1);
    }

    void TraversabilityCostFunction::predictRectangles(base_local_planner::SimpleTrajectoryGenerator generator)
    {   
        // Create a vector of inputs
        std::vector<torch::jit::IValue> rectangles;

        // Create a vector to store the rectangular regions
        std::vector<at::Tensor> rectangles_vector;

        // Create a trajectory object
		base_local_planner::Trajectory traj;

        // Define a variable to store the success state of the trajectory generation
		bool generation_success;

        // Define a variable to count the number of trajectories
        int nb_trajectories = 0;
        
        nb_rectangles_vector_.clear();

        // Go through all the trajectories
		while (generator.hasMoreTrajectories())
		{
            // Generate the next trajectory
        	generation_success = generator.nextTrajectory(traj);

            // Define a variable to store the number of rectangles in the current trajectory
            int nb_rectangles = 0;

			// Check if the trajectory was successfully generated
			if (generation_success == false)
				continue;
            
            nb_trajectories++;

            // Create an array to store the previous pair of points
            float previous_pair_image[4];
            bool is_previous_pair_image_initialized = false;

            // Go through the points of the trajectory
            for (int i = 0; i < traj.getPointsSize() - 1; i++)
            {
                // Define variables to store the current pose of the robot in the robot frame
                double x, y, th;

                // Get the coordinates of the current point of the trajectory
                traj.getPoint(i, x, y, th);

                // Compute the distances between the wheels and the robot's origin
                double delta_X = L_*sin(th)/2;
                double delta_Y = L_*cos(th)/2;

                // Compute the positions of the outer points of the two front wheels
                // (z coordinate is set to 0 because we assume the ground is flat)
                cv::Point3f current_point_left_robot = cv::Point3f(x-delta_X, y+delta_Y, 0.0);
                cv::Point3f current_point_right_robot = cv::Point3f(x+delta_X, y-delta_Y, 0.0);

                // Define vectors to store the coordinates of the current pair of
                // point in the robot frame and in the image plan
                std::vector<cv::Point3f> current_pair_robot;
                std::vector<cv::Point2f> current_pair_image;

                // Add the two points to the vector
                current_pair_robot.push_back(current_point_left_robot);
                current_pair_robot.push_back(current_point_right_robot);

                // Compute the coordinates of those two points in the image plan
                cv::projectPoints(current_pair_robot, cam_to_robot_rotation_, cam_to_robot_translation_, K_, cv::Mat(), current_pair_image);

                // Keep only pairs of points contained in the image
                if (current_pair_image[0].x > 0 && current_pair_image[0].x < IMAGE_W_ &&
                    current_pair_image[0].y > 0 && current_pair_image[0].y < IMAGE_H_ &&
                    current_pair_image[1].x > 0 && current_pair_image[1].x < IMAGE_W_ &&
                    current_pair_image[1].y > 0 && current_pair_image[1].y < IMAGE_H_)
                {   
                    // If this is the first pair of points, store it and continue
                    if (!is_previous_pair_image_initialized)
                    {
                        // Initialize the previous pair of points
                        previous_pair_image[0] = current_pair_image[0].x;
                        previous_pair_image[1] = current_pair_image[0].y;
                        previous_pair_image[2] = current_pair_image[1].x;
                        previous_pair_image[3] = current_pair_image[1].y;

                        // Set the flag to true
                        is_previous_pair_image_initialized = true;

                        continue;
                    }
                    // Get the bounding box given the two pairs of points
                    cv::Mat rectangle_image = cv_ptr_->image(cv::Range(std::min(current_pair_image[0].y, current_pair_image[1].y),
                                                                       std::max(previous_pair_image[1], previous_pair_image[3])),
                                                             cv::Range(std::min(current_pair_image[0].x, previous_pair_image[0]),
                                                                       std::max(current_pair_image[1].x, previous_pair_image[2])));

                    // Resize the image
                    cv::resize(rectangle_image, rectangle_image, cv::Size(210, 70));

                    // Convert the image to float and set the range to [0, 1]
                    rectangle_image.convertTo(rectangle_image, CV_32FC3, 1.0f / 255.0f);

                    // Disable gradient computation 
                    at::NoGradGuard no_grad;

                    // Convert the image to tensor
                    at::Tensor rectangle_tensor = at::from_blob(rectangle_image.data,
                                                                {1,
                                                                rectangle_image.rows,
                                                                rectangle_image.cols,
                                                                rectangle_image.channels()},
                                                                at::kFloat);
                    rectangle_tensor = rectangle_tensor.permute({0, 3, 1, 2});

                    // Make a copy of the tensor and store it in the vector
                    rectangles_vector.push_back(rectangle_tensor.clone());

                    // Store the current pair of points as the previous pair of points
                    previous_pair_image[0] = current_pair_image[0].x;
                    previous_pair_image[1] = current_pair_image[0].y;
                    previous_pair_image[2] = current_pair_image[1].x;
                    previous_pair_image[3] = current_pair_image[1].y;

                    // Increment the number of rectangles
                    nb_rectangles++;
                }
            }
            nb_rectangles_vector_.push_back(nb_rectangles);
		}
        // Concatenate and normalize the tensors and send them to the GPU
        rectangles.push_back(normalize_transform_(at::cat(rectangles_vector)).to(device_));

        // Execute the model and turn its output into a tensor
        predicted_costs_rectangles_ = at::softmax(model_.forward(rectangles).toTensor(), /*dim*/1);

        // Set the number of trajectories
        nb_trajectories_ = nb_trajectories;
    }
}
