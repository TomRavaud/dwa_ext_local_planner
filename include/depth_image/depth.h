// Include guards to prevent double declarations (eq #pragma once)
#ifndef DEPTH_
#define DEPTH_


#include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include <iostream>


class Depth
{
public:
    Depth(cv::Mat depth_image);

    cv::Mat computeNormals();

    cv::Mat getDepth() const { return depth_image_; }

    void setDepth(cv::Mat depth_image) { depth_image_ = depth_image; }

private:
    cv::Mat depth_image_;
};

void displayImage(cv::Mat image);
cv::Mat convert_range(
    cv::Mat image,
    float min1,
    float max1,
    float min2,
    float max2);

#endif  // DEPTH_
