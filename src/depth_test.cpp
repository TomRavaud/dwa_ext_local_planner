#include "depth_image/depth.h"


int main()
{   
    // Load the depth map
    cv::Mat depth_image = cv::imread(
        "/home/tom/Traversability-Tom/catkin_ws/src/dwa_ext_local_planner/bindings/depth_image_zed.tiff",
        cv::IMREAD_ANYDEPTH);

    Depth depth(depth_image);

    depth_image = convert_range(depth_image, 0.7, 7, 0, 255);

    displayImage(depth_image);

    cv::waitKey(0);

    // // Apply a bilateral filter to smooth the image
    // cv::Mat filtered_depth_image;
    // // For simplicity, you can set the 2 sigma values to be the same.
    // // If they are small (< 10), the filter will not have much effect,
    // // whereas if they are large (> 150), they will have a very strong
    // // effect, making the image look "cartoonish".
    // cv::bilateralFilter(depth_image, filtered_depth_image, 5, 75, 75);

    // Apply Sobel filters to find the image gradients in the x and
    // y directions
    // cv::Mat grad_x, grad_y;
    // cv::Sobel(depth_image, grad_x, CV_32F, 1, 0, 3);
    // cv::Sobel(depth_image, grad_y, CV_32F, 0, 1, 3);
    
    return 0;
}
