#include "dwa_ext_local_planner/traversability_cost_function.h"

#include <time.h>
#include <cstdlib>


namespace dwa_ext_local_planner {

	void TraversabilityCostFunction::callbackImage(const sensor_msgs::ImageConstPtr& image)
	{   
		// To convert ROS Image type into a CvImage
    	cv_bridge::CvImagePtr cvPtr;

    	// Convert and copy the ROS Image into a CvImage
    	cvPtr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

		cv::imshow("Preview", cvPtr->image);
    	cv::waitKey(5);
	}

    TraversabilityCostFunction::TraversabilityCostFunction()
    {   
		ros::NodeHandle nh;
        std::srand((unsigned)time(NULL));

		sub_image_ = nh.subscribe("camera1/image_raw", 1, &dwa_ext_local_planner::TraversabilityCostFunction::callbackImage, this);
    }

    TraversabilityCostFunction::~TraversabilityCostFunction(){}

    bool TraversabilityCostFunction::prepare() {
        return true;
    }

    double TraversabilityCostFunction::scoreTrajectory(base_local_planner::Trajectory &traj) {
        double cost = (double) rand()/RAND_MAX;
        return cost;
    }
}
