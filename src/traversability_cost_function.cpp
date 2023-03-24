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

        // // Create a vector of inputs
        // std::vector<torch::jit::IValue> inputs;

        // // Create a vector to store the rectangular regions
        // std::vector<torch::Tensor> rectangles_vector;

        // for (int i = 8; i < 9; i++)
        // {
        // // Load the test image
        // cv::Mat image = cv::imread("/home/tom/Traversability-Tom/Husky/src/dwa_ext_local_planner/rectangle"+std::to_string(i)+".png");

        // // Resize the image
        // cv::Mat image_float;
        // cv::resize(image, image_float, cv::Size(210, 70));

        // // Convert the image to float
        // image_float.convertTo(image_float, CV_32FC3, 1.0f / 255.0f);

        // // Convert the image to tensor
        // at::Tensor tensor = torch::from_blob(image_float.data, {1, image_float.rows, image_float.cols, image_float.channels()}, at::kFloat);
        // tensor = tensor.permute({0, 3, 1, 2});

        // // Normalize the image
        // tensor = normalize_transform_(tensor);

        // // Send the tensor to the GPU
        // tensor = tensor.to(device_);
        // // std::cout << tensor.options() << std::endl;

        // // Add the tensor to the input vector
        // rectangles_vector.push_back(tensor);
        // }

        // // Add the tensor to the input vector
        // inputs.push_back(torch::cat(rectangles_vector));

        // // Execute the model and turn its output into a tensor
        // at::Tensor output = at::softmax(model_.forward(inputs).toTensor(), /*dim*/1);
        // std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/10) << '\n';
    }

    TraversabilityCostFunction::~TraversabilityCostFunction(){}

    bool TraversabilityCostFunction::prepare() {
        return true;
    }

    double TraversabilityCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj)
    {
        // Create a vector of inputs
        std::vector<torch::jit::IValue> rectangles;

        // Create a vector to store the rectangular regions
        std::vector<torch::Tensor> rectangles_vector;

        // Create a vector to store the previous pair of points
        std::vector<cv::Point2d> previous_pair_image;

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
                // If this is the first pair of points, store it and continue
                if (previous_pair_image.size() == 0)
                {
                    previous_pair_image.push_back(current_pair_image[0]);
                    previous_pair_image.push_back(current_pair_image[1]);
                    continue;
                }

                // Get the bounding box given the two pairs of points
                cv::Mat rectangle = cv_ptr_->image(cv::Range(std::min(current_pair_image[0].y, current_pair_image[1].y),
                                                             std::max(previous_pair_image[0].y, previous_pair_image[1].y)),
                                                   cv::Range(std::min(current_pair_image[0].x, previous_pair_image[0].x),
                                                             std::max(current_pair_image[1].x, previous_pair_image[1].x)));
                
                // Check that the rectangle is not empty
                assert(!rectangle.empty());

                // Resize the image
                cv::Mat rectangle_float;
                cv::resize(rectangle, rectangle_float, cv::Size(210, 70));

                // cv::imwrite("/home/tom/Traversability-Tom/Husky/src/dwa_ext_local_planner/rectangle"+std::to_string(i)+".png", rectangle_float);

                // Convert the image to float
                rectangle_float.convertTo(rectangle_float, CV_32FC3, 1.0f / 255.0f);

                // Disable gradient computation 
                torch::NoGradGuard no_grad;

                // Convert the image to tensor
                at::Tensor rectangle_tensor = torch::from_blob(rectangle_float.data, {1, rectangle_float.rows, rectangle_float.cols, rectangle_float.channels()}, at::kFloat);
                rectangle_tensor = rectangle_tensor.permute({0, 3, 1, 2});

                // Normalize the image
                rectangle_tensor = normalize_transform_(rectangle_tensor);

                // Send the tensor to the GPU
                rectangle_tensor = rectangle_tensor.to(device_);
                // std::cout << tensor.options() << std::endl;

                // Add the tensor to the input vector
                rectangles_vector.push_back(rectangle_tensor);

                // Store the current pair of points as the previous pair of points
                previous_pair_image.clear();
                previous_pair_image.push_back(current_pair_image[0]);
                previous_pair_image.push_back(current_pair_image[1]);
            }
        }

        // If there are no rectangles, return -1
        if (rectangles_vector.size() == 0)
        {
            return -1;
        }

        // Concatenate the tensors
        rectangles.push_back(torch::cat(rectangles_vector));

        // Execute the model and turn its output into a tensor
        at::Tensor output = at::softmax(model_.forward(rectangles).toTensor(), /*dim*/1);
        // std::cout << output.slice(/*dim=*/1, /*start=*/0, /*end=*/10) << '\n';
        
        // Compute the expected costs over the bins
        at::Tensor result = torch::mm(output, bins_midpoints_);

        // Get the maximum cost
        at::Tensor cost_tensor = at::max(result);
        double cost = cost_tensor.item<double>();

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

            // std::cout << "green: " << green << std::endl;
            // std::cout << "red: " << red << std::endl;

            // Draw the polygon
            cv::fillPoly(overlay, points_vector, cv::Scalar(0, green, red));

            // Weighted sum of the original image and the overlay to create transparency
            double transparency = 0.5;  // Specify the level of transparency
            cv::addWeighted(overlay, transparency, image_to_display, 1 - transparency, 0, image_to_display);
        }
        // Display the current image
		cv::imshow("Preview", image_to_display);
    	cv::waitKey(1);
    }
}
