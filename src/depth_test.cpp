#include "depth_image/depth.h"


int main()
{   
    // Load the depth map
    cv::Mat depth_image = cv::imread(
        "/home/tom/Traversability-Tom/catkin_ws/src/dwa_ext_local_planner/media/depth_image.tiff",
        cv::IMREAD_ANYDEPTH);

    Depth depth(depth_image, 0.7, 7);

    // Set the internal calibration matrix
    cv::Mat K_ = (cv::Mat_<double>(3, 3) << 1067, 0, 943,
                                            0, 1067, 521,
                                            0, 0, 1);

    depth.computeNormals(K_, 10);

    depth.displayDepth();
    depth.displayNormals();

    cv::waitKey(0);

    return 0;
}
