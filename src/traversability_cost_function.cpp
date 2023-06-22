#include "dwa_ext_local_planner/traversability_cost_function.h"


namespace dwa_ext_local_planner {

	void TraversabilityCostFunction::callbackImage(
        const sensor_msgs::ImageConstPtr& image)
	{
		// Convert and copy the ROS Image into a CvImage
        // We use the BGR order to keep consistency with OpenCV
    	cv_ptr_ = cv_bridge::toCvCopy(image,
                                      sensor_msgs::image_encodings::BGR8);
	}

    TraversabilityCostFunction::TraversabilityCostFunction() :
        device_(torch::cuda::is_available() ? torch::kCUDA : torch::kCPU)
    {
		ros::NodeHandle nh;

        // Initialize the random number generator
        std::srand((unsigned)time(NULL));

        // Load parameters from the parameter server
        nh.getParam("/move_base/DWAExtPlannerROS/Traversability/L",
                    L_);
        nh.getParam("/move_base/DWAExtPlannerROS/Traversability/W",
                    IMAGE_W_);
        nh.getParam("/move_base/DWAExtPlannerROS/Traversability/H",
                    IMAGE_H_);
        nh.getParam("/move_base/DWAExtPlannerROS/Traversability/patch_distance",
                    PATCH_DISTANCE_);
        nh.getParam("/move_base/DWAExtPlannerROS/Traversability/rectangle_ratio",
                    RECTANGLE_RATIO_);
        nh.getParam("/move_base/DWAExtPlannerROS/Traversability/nb_rectangles_max",
                    NB_RECTANGLES_MAX_);
        nh.getParam("/move_base/DWAExtPlannerROS/Traversability/display_absolute_cost",
                    DISPLAY_ABSOLUTE_COST_);
        nh.getParam("/move_base/DWAExtPlannerROS/Traversability/cost_max",
                    COST_MAX_);
        nh.getParam("/move_base/DWAExtPlannerROS/Traversability/cost_min",
                    COST_MIN_);

        // Read the name of the camera image topic
        std::string image_topic {};
        nh.getParam("/move_base/DWAExtPlannerROS/Traversability/image_topic",
                    image_topic);

        // Initialize the subscriber to the camera image topic
		sub_image_ = nh.subscribe(
            image_topic,
            1,
            &dwa_ext_local_planner::TraversabilityCostFunction::callbackImage,
            this);

        // Load the translation vector from the robot frame to the camera
        // frame
        std::vector<float> robot_to_cam_translation {};
        nh.getParam(
            "/move_base/DWAExtPlannerROS/Traversability/robot_to_cam_translation",
            robot_to_cam_translation);
        robot_to_cam_translation_ =
            cv::Mat(robot_to_cam_translation).reshape(0, 3);

        // Load the tilt angle of the camera
        nh.getParam(
            "/move_base/DWAExtPlannerROS/Traversability/alpha", alpha_);

        // Set the rotation matrix to rotate the robot frame to the
        // camera frame
        robot_to_cam_rotation_ =
            (cv::Mat_<double>(3, 3) << 0.0, sin(alpha_), cos(alpha_),
                                       -1.0, 0.0, 0.0,
                                       0.0, -cos(alpha_), sin(alpha_));

        // Set the rotation matrix to rotate the camera frame to the
        // robot frame
        cv::invert(robot_to_cam_rotation_, cam_to_robot_rotation_);

        // Set the translation vector to translate the robot frame to the
        // camera frame
        cam_to_robot_translation_ =
            -cam_to_robot_rotation_*robot_to_cam_translation_;

        // Load the internal calibration matrix
        std::vector<float> K {};
        nh.getParam("/move_base/DWAExtPlannerROS/Traversability/K", K);
        K_ = cv::Mat(K).reshape(0, 3);

        // Device
        ROS_INFO("Device for NN inference is %s", device_.str().c_str());

        // Get the path of the model definition file
        std::string model_definition_file {};
        nh.getParam(
            "/move_base/DWAExtPlannerROS/Traversability/model_definition_file",
            model_definition_file);
        
        // Load the model
        model_ = torch::jit::load(model_definition_file);

        // Send the model to the GPU
        model_.to(device_);

        // Load the bins midpoints from the parameter server
        std::vector<float> midpoints {};
        nh.getParam(
            "/move_base/DWAExtPlannerROS/Traversability/bins_midpoints",
            midpoints);

        // Convert the bins midpoints to a tensor
        bins_midpoints_ = torch::from_blob(midpoints.data(),
                                           {(long int)std::size(midpoints), 1},
                                           torch::kFloat32);

        // Send the bins midpoints to the GPU
        bins_midpoints_ = bins_midpoints_.to(device_);
    }

    TraversabilityCostFunction::~TraversabilityCostFunction(){}

    inline bool TraversabilityCostFunction::prepare()
    {
        return true;
    }

    inline double TraversabilityCostFunction::costInterpolation(double vth)
    {
        // Get the number of angular velocities
        int nb_values = vth_values_.size();

        // Define a variable to store the cost of the trajectory
        double cost {};

        // Interpolate the cost
        if (vth < vth_values_[0])
        {
            cost = cost_values_[0];
        }
        else if (vth > vth_values_[nb_values - 1])
        {
            cost = cost_values_[nb_values - 1];
        }
        else
        {
            for (int i { 0 }; i < nb_values - 1; i++)
            {
                if (vth >= vth_values_[i] && vth < vth_values_[i+1])
                {   
                    // Set the cost as the maximum of the two neighboring
                    // costs
                    cost = std::max(cost_values_[i], cost_values_[i+1]);
                    break;
                }
            }
        }

        return cost;
    }

    inline double TraversabilityCostFunction::scoreTrajectory(
        base_local_planner::Trajectory &traj)
    {   
        // Compute the cost of the trajectory
        double cost { costInterpolation(traj.thetav_) };

        return cost;
    }

    void TraversabilityCostFunction::displayTrajectoriesAndCosts(
        std::vector<base_local_planner::Trajectory> &trajs)
    {   
        // Get the current image (there is no need to convert it to RGB since
        // it is the default OpenCV format)
        cv::Mat image_to_display = cv_ptr_->image.clone();

        double cost_min, cost_max;

        if (DISPLAY_ABSOLUTE_COST_)
        {
            // Get the minimum and maximum costs on the training set
            cost_min = COST_MIN_;
            cost_max = COST_MAX_;
        }
        else
        {   
            // Compute the maximum and minimum costs among the candidate
            // trajectories
            cost_min = COST_MAX_;
            cost_max = COST_MIN_;

            for (int i { 0 }; i < trajs.size(); i++)
            {
                // Get the cost of the trajectory
                double cost { trajs[i].cost_ };

                // Discard the trajectory if the cost is negative
                if (cost < 0.0)
                    continue;

                // Update the minimum cost
                if (cost < cost_min)
                    cost_min = cost;

                // Update the maximum cost
                if (cost > cost_max)
                    cost_max = cost;
            }
        }

        // Display the costs of the trajectories
        for (int i { 0 }; i < trajs.size(); i++)
        {
            std::vector<cv::Point2d> previous_pair_image;

            // Create vectors to store the points of the trajectory
            std::vector<cv::Point> points_left;
            std::vector<cv::Point> points_right;
            std::vector<cv::Point> points;

            // Create a point to store the previous position of the robot in
            // the robot frame
            cv::Point2d previous_point_robot;

            // std::cout << trajs[i].xv_ << "\n";
            // std::cout << trajs[i].yv_ << "\n";
            // std::cout << trajs[i].thetav_ << "\n";

            for (int j { 0 }; j < trajs[i].getPointsSize(); j++)
            {
                // Define variables to store the current pose of the robot in
                // the robot frame
                double x, y, th;

                // Get the coordinates of the current point of the trajectory
                trajs[i].getPoint(j, x, y, th);

                // Compute the distances between the wheels and the robot's
                // origin
                double delta_X { L_*sin(th)/2 };
                double delta_Y { L_*cos(th)/2 };

                // Compute the positions of the outer points of the two
                // front wheels
                // (z coordinate is set to 0 because we assume the ground
                // is flat)
                cv::Point3d current_point_left_robot { x-delta_X,
                                                       y+delta_Y,
                                                       0.0 };
                cv::Point3d current_point_right_robot { x+delta_X,
                                                        y-delta_Y,
                                                        0.0 };

                // Define vectors to store the coordinates of the current pair
                // of points in the robot frame and in the image plan
                std::vector<cv::Point3d> current_pair_robot;
                std::vector<cv::Point2d> current_pair_image;

                // Add the two points to the vector
                current_pair_robot.push_back(current_point_left_robot);
                current_pair_robot.push_back(current_point_right_robot);

                // Compute the coordinates of those two points in the image
                // plan
                cv::projectPoints(current_pair_robot,
                                  cam_to_robot_rotation_,
                                  cam_to_robot_translation_,
                                  K_,
                                  cv::Mat(),
                                  current_pair_image);

                // Keep only pairs of points contained in the image
                if (current_pair_image[0].x > 0 &&
                    current_pair_image[0].x < IMAGE_W_ &&
                    current_pair_image[0].y > 0 &&
                    current_pair_image[0].y < IMAGE_H_ &&
                    current_pair_image[1].x > 0 &&
                    current_pair_image[1].x < IMAGE_W_ &&
                    current_pair_image[1].y > 0 &&
                    current_pair_image[1].y < IMAGE_H_)
                {
                    // Draw circles at the 2D points on the image
                    // cv::circle(image_to_display,
                    //            current_pair_image[0],
                    //            5,
                    //            cv::Scalar(0, 0, 255),
                    //            -1);
                    // cv::circle(image_to_display,
                    //            current_pair_image[1],
                    //            5, cv::Scalar(0, 0, 255),
                    //            -1);

                    points_left.push_back(cv::Point(current_pair_image[0].x,
                                                    current_pair_image[0].y));
                    points_right.push_back(cv::Point(current_pair_image[1].x,
                                                     current_pair_image[1].y));

                    // If there is no previous pair of points, store the
                    // current pair of points
                    if (previous_pair_image.size() == 0)
                    {
                        previous_pair_image.push_back(current_pair_image[0]);
                        previous_pair_image.push_back(current_pair_image[1]);

                        previous_point_robot.x = x;
                        previous_point_robot.y = y;

                        continue;
                    }

                    // Compute the distance between the current point and the
                    // previous point
                    double distance2 { pow(x-previous_point_robot.x, 2) +
                                       pow(y-previous_point_robot.y, 2) };

                    if (distance2 > pow(PATCH_DISTANCE_, 2))
                    {
                        // Compute the coordinates of the bounding box
                        // corners
                        int min_x = std::min(current_pair_image[0].x,
                                             previous_pair_image[0].x);
                        int min_y = std::min(current_pair_image[0].y,
                                             current_pair_image[1].y);
                        int max_x = std::max(current_pair_image[1].x,
                                             previous_pair_image[1].x);
                        int max_y = std::max(previous_pair_image[0].y,
                                             previous_pair_image[1].y);

                        // Correct the dimensions of the rectangle to respect
                        // the height-width ratio
                        int rectangle_width { max_x - min_x };
                        int rectangle_height { max_y - min_y };

                        int min_x_rectangle, min_y_rectangle, max_x_rectangle, max_y_rectangle;
                        
                        if (rectangle_width < RECTANGLE_RATIO_*rectangle_height)
                        {   
                            // Height of the rectangular regions to be
                            // eliminated on the right and left of the
                            // patch
                            int delta = (rectangle_height - rectangle_width/RECTANGLE_RATIO_)/2;

                            // Coordinates of the vertices of the patch to keep
                            min_y_rectangle = min_y + delta;
                            max_y_rectangle = max_y - delta;
                            min_x_rectangle = min_x;
                            max_x_rectangle = max_x;
                        }

                        else
                        {
                            // Width of the rectangular regions to be
                            // eliminated at the top and bottom of the
                            // patch
                            int delta = (rectangle_width - RECTANGLE_RATIO_*rectangle_height)/2;
                            
                            // Coordinates of the vertices of the patch to keep
                            min_x_rectangle = min_x + delta;
                            max_x_rectangle = max_x - delta;
                            min_y_rectangle = min_y;
                            max_y_rectangle = max_y;
                        }

                        // Draw the box
                        cv::rectangle(
                            image_to_display,
                            cv::Point(
                                min_x_rectangle,
                                min_y_rectangle),
                            cv::Point(
                                max_x_rectangle,
                                max_y_rectangle),
                            cv::Scalar(0, 255, 0));

                        // Store the current pair of points as the previous pair
                        // of points
                        previous_pair_image.clear();
                        previous_pair_image.push_back(current_pair_image[0]);
                        previous_pair_image.push_back(current_pair_image[1]);

                        // Store the current point as the previous point
                        previous_point_robot.x = x;
                        previous_point_robot.y = y;
                    }
                }
            }
            std::cout << "Trajectory " << i
                      << " cost: " << trajs[i].cost_ << std::endl;

            // Fill the vectors of points with the left and right points
            points.insert(points.end(),
                          points_left.begin(),
                          points_left.end());
            std::reverse(points_right.begin(), points_right.end());
            points.insert(points.end(),
                          points_right.begin(),
                          points_right.end());

            if (points.size() == 0)
                continue;
            
            // Create a vector of vectors of points to be able to use the
            // fillPoly function
            std::vector<std::vector<cv::Point>> points_vector = {points};

            // Create a copy of the image to display
            cv::Mat overlay = image_to_display.clone();

            // Compute the green and red values to display the cost
            double green = 255/(cost_min - cost_max)*trajs[i].cost_
                           + 255*cost_max/(cost_max - cost_min);
            double red = 255 - green;

            // Draw the polygon
            cv::fillPoly(overlay, points_vector, cv::Scalar(0, green, red));

            // Weighted sum of the original image and the overlay to create
            // transparency
            double transparency = 0.3;  // Specify the level of transparency
            cv::addWeighted(overlay,
                            transparency,
                            image_to_display,
                            1-transparency,
                            0,
                            image_to_display);
        }
        // Resize the image to display
        cv::resize(image_to_display, image_to_display, cv::Size(1280, 720));

        // Display the current image
		cv::imshow("Preview", image_to_display);
    	cv::waitKey(1);
    }

    void TraversabilityCostFunction::predictRectangles(
        base_local_planner::SimpleTrajectoryGenerator generator,
        std::vector<base_local_planner::Trajectory>* all_explored)
    {   
        auto time_start = std::chrono::high_resolution_clock::now();

        // Create a vector of inputs
        std::vector<torch::jit::IValue> inputs;

        // Create a vector to store the rectangular regions
        std::vector<at::Tensor> rectangles_vector;

        // Create a vector to store the velocities
        std::vector<at::Tensor> velocities_vector;

        // Create a trajectory object
		base_local_planner::Trajectory traj;

        // Define a variable to store the success state of the trajectory
        // generation
		bool generation_success;

        // Define a variable to count the number of trajectories
        int nb_trajectories { 0 };
        
        // Define a vector to store the number of rectangles per trajectory
        std::vector<int> nb_rectangles_vector;

        cost_values_.clear();
        vth_values_.clear();

        // Go through all the trajectories
		while (generator.hasMoreTrajectories())
		{
            // Generate the next trajectory
        	generation_success = generator.nextTrajectory(traj);

            // Define a variable to store the number of rectangles in the
            // current trajectory
            int nb_rectangles { 0 };

			// Check if the trajectory was successfully generated
			if (generation_success == false)
				continue;

            // Append the trajectory to the vector of all explored trajectories
            // (for visualization purposes only)
            if (all_explored != NULL)
                all_explored->push_back(traj);

            // Append the angular velocity of the current trajectory to the
            // vector of angular velocities
            vth_values_.push_back(traj.thetav_);
            
            nb_trajectories++;

            // Create an array to store the previous pair of points
            double previous_pair_image[4];
            bool is_previous_pair_image_initialized = false;

            // Create a point to store the previous position of the robot
            // in the robot frame
            cv::Point2d previous_point_robot;

            // Go through the points of the trajectory
            for (int i { 0 }; i < traj.getPointsSize() - 1; i++)
            {
                // Define variables to store the current pose of the robot in
                // the robot frame
                double x, y, th;

                // Get the coordinates of the current point of the trajectory
                traj.getPoint(i, x, y, th);

                // Compute the distances between the wheels and the robot's
                // origin
                double delta_X { L_*sin(th)/2 };
                double delta_Y { L_*cos(th)/2 };

                // Compute the positions of the outer points of the two
                // front wheels
                // (z coordinate is set to 0 because we assume the ground
                // is flat)
                cv::Point3d current_point_left_robot { x-delta_X,
                                                       y+delta_Y,
                                                       0.0 };
                cv::Point3d current_point_right_robot { x+delta_X,
                                                        y-delta_Y,
                                                        0.0 };

                // Define vectors to store the coordinates of the current
                // pair of point in the robot frame and in the image plan
                std::vector<cv::Point3d> current_pair_robot;
                std::vector<cv::Point2d> current_pair_image;

                // Add the two points to the vector
                current_pair_robot.push_back(current_point_left_robot);
                current_pair_robot.push_back(current_point_right_robot);

                // Compute the coordinates of those two points in the image
                // plan
                cv::projectPoints(current_pair_robot,
                                  cam_to_robot_rotation_,
                                  cam_to_robot_translation_,
                                  K_,
                                  cv::Mat(),
                                  current_pair_image);

                // Keep only pairs of points contained in the image
                if (current_pair_image[0].x > 0 &&
                    current_pair_image[0].x < IMAGE_W_ &&
                    current_pair_image[0].y > 0 &&
                    current_pair_image[0].y < IMAGE_H_ &&
                    current_pair_image[1].x > 0 &&
                    current_pair_image[1].x < IMAGE_W_ &&
                    current_pair_image[1].y > 0 &&
                    current_pair_image[1].y < IMAGE_H_)
                {   
                    // If it is the first pair of points, store it and
                    // continue
                    if (!is_previous_pair_image_initialized)
                    {
                        // Initialize the previous pair of points
                        previous_pair_image[0] = current_pair_image[0].x;
                        previous_pair_image[1] = current_pair_image[0].y;
                        previous_pair_image[2] = current_pair_image[1].x;
                        previous_pair_image[3] = current_pair_image[1].y;

                        previous_point_robot.x = x;
                        previous_point_robot.y = y;

                        // Set the flag to true
                        is_previous_pair_image_initialized = true;

                        continue;
                    }

                    // Compute the distance between the current point and the
                    // previous point
                    double distance2 { pow(x-previous_point_robot.x, 2) +
                                       pow(y-previous_point_robot.y, 2) };


                    if (distance2 > pow(PATCH_DISTANCE_, 2) &&
                        nb_rectangles < NB_RECTANGLES_MAX_)
                    {
                        // Compute the coordinates of the bounding box
                        // corners
                        int min_x = std::min(current_pair_image[0].x,
                                             previous_pair_image[0]);
                        int min_y = std::min(current_pair_image[0].y,
                                             current_pair_image[1].y);
                        int max_x = std::max(current_pair_image[1].x,
                                             previous_pair_image[2]);
                        int max_y = std::max(previous_pair_image[1],
                                             previous_pair_image[3]);

                        // Correct the dimensions of the rectangle to respect
                        // the height-width ratio
                        int rectangle_width { max_x - min_x };
                        int rectangle_height { max_y - min_y };

                        int min_x_rectangle, min_y_rectangle, max_x_rectangle, max_y_rectangle;
                        
                        if (rectangle_width < RECTANGLE_RATIO_*rectangle_height)
                        {   
                            // Height of the rectangular regions to be
                            // eliminated on the right and left of the
                            // patch
                            int delta = (rectangle_height - rectangle_width/RECTANGLE_RATIO_)/2;

                            // Coordinates of the vertices of the patch to keep
                            min_y_rectangle = min_y + delta;
                            max_y_rectangle = max_y - delta;
                            min_x_rectangle = min_x;
                            max_x_rectangle = max_x;
                        }

                        else
                        {
                            // Width of the rectangular regions to be
                            // eliminated at the top and bottom of the
                            // patch
                            int delta = (rectangle_width - RECTANGLE_RATIO_*rectangle_height)/2;
                            
                            // Coordinates of the vertices of the patch to keep
                            min_x_rectangle = min_x + delta;
                            max_x_rectangle = max_x - delta;
                            min_y_rectangle = min_y;
                            max_y_rectangle = max_y;
                        }

                        // Get the box given the two pairs of points
                        cv::Mat rectangle_image = cv_ptr_->image(
                            cv::Range(
                                min_y_rectangle,
                                max_y_rectangle),
                            cv::Range(
                                min_x_rectangle,
                                max_x_rectangle));

                        // Resize the image
                        cv::resize(rectangle_image,
                                   rectangle_image,
                                   cv::Size(210, 70));

                        // Convert the BGR image to RGB
                        cv::cvtColor(rectangle_image,
                                     rectangle_image,
                                     CV_BGR2RGB);

                        // Convert the image to float and set the range to [0, 1]
                        rectangle_image.convertTo(rectangle_image,
                                                  CV_32FC3,
                                                  1.0f / 255.0f);

                        // Disable gradient computation 
                        at::NoGradGuard no_grad;

                        // Convert the image to tensor and permute the dimensions
                        // to match the network's input shape
                        at::Tensor rectangle_tensor = at::from_blob(
                            rectangle_image.data,
                            { 1,
                              rectangle_image.rows,
                              rectangle_image.cols,
                              rectangle_image.channels() },
                            at::kFloat);
                        rectangle_tensor = rectangle_tensor.permute(
                            { 0, 3, 1, 2 });

                        // Make a copy of the tensor and store it in the vector
                        rectangles_vector.push_back(rectangle_tensor.clone());

                        velocities_vector.push_back(
                            at::tensor({ traj.xv_ }));

                        // Store the current pair of points as the previous pair
                        // of points
                        previous_pair_image[0] = current_pair_image[0].x;
                        previous_pair_image[1] = current_pair_image[0].y;
                        previous_pair_image[2] = current_pair_image[1].x;
                        previous_pair_image[3] = current_pair_image[1].y;

                        // Store the current robot position as the previous
                        // robot position
                        previous_point_robot.x = x;
                        previous_point_robot.y = y;

                        // Increment the number of rectangles
                        nb_rectangles++;
                    }

                    else if (nb_rectangles == NB_RECTANGLES_MAX_)
                        break;
                }
            }
            
            // If the trajectory contains less than NB_RECTANGLES_MAX_
            // rectangles, it will not be possible to pass the sequence
            // of rectangles to the recurrent network
            if (nb_rectangles < NB_RECTANGLES_MAX_)
                ROS_WARN("The trajectory contains less than %d rectangles. "
                         "It will not be possible to pass the sequence of "
                         "rectangles to the recurrent network.",
                         NB_RECTANGLES_MAX_);

            nb_rectangles_vector.push_back(nb_rectangles);
		}
        // Concatenate and normalize the tensors and send them to the GPU
        inputs.push_back(
            normalize_transform_(at::cat(rectangles_vector)).to(device_));
        
        // Concatenate, add a dimension and send the velocities tensor
        // to the GPU
        inputs.push_back(at::cat(velocities_vector).to(at::kFloat)
            .unsqueeze_(1).to(device_));

        // Compute the expected costs over the bins
        at::Tensor expected_costs_rectangles;
        expected_costs_rectangles = at::mm(
            at::softmax(model_.forward(inputs).toTensor(), /*dim*/1),
            bins_midpoints_);

        // Initialize the index of the rectangle
        int index_rectangle { 0 };

        for (int i { 0 }; i < nb_trajectories; i++)
        {
            // Get the number of rectangles in the current trajectory
            int nb_rectangles = nb_rectangles_vector[i];

            double cost;
            
            // If we cannot assess the traversability of a trajectory,
            // discard it by setting its cost to -1
            if (nb_rectangles == 0)
                cost = -1;

            else
                // Get the maximum cost
                cost = at::max(
                    expected_costs_rectangles.narrow(
                        0,
                        index_rectangle,
                        nb_rectangles)).item<double>();

            cost_values_.push_back(cost);

            // Set the cost of each trajectory
            // (for visualization purposes)
            if (all_explored != NULL)
                // Need to dereference the pointer to modify its data
                (*all_explored)[i].cost_ = cost;

            // Increment the index of the rectangle
            index_rectangle += nb_rectangles;
        }
    }
}
