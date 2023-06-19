// Include guards to prevent double declarations (eq #pragma once)
#ifndef DEPTH_
#define DEPTH_


#include <opencv2/opencv.hpp>
#include <iostream>
#include <limits>


class Depth
{
public:
    /**
     * @brief Construct a new Depth object
     * 
     * @param depth The depth image
     * @param depth_min Minimum depth value the stereo camera can measure
     * (in meters)
     * @param depth_max Maximum depth value the stereo camera can measure
     * (in meters)
     */
    Depth(cv::Mat depth, double depth_min, double depth_max);

    /**
     * @brief Estimate the surface normals from a depth image
     * 
     * @param K The internal calibration matrix
     * @param gradient_threshold The threshold for the gradient magnitude
     * to be considered valid
     */
    void computeNormals(cv::Mat K, double gradient_threshold);

    /**
     * @brief Display the depth image
     * 
     */
    void displayDepth() const;

    /**
     * @brief Display the surface normals
     * 
     */
    void displayNormals() const;

    /**
     * @brief Get the depth image
     * 
     * @return cv::Mat The depth image
     */
    inline cv::Mat getDepth() const { return depth_; }

    /**
     * @brief Get the surface normals
     * 
     * @return cv::Mat The surface normals
     */
    inline cv::Mat getNormals() const { return normal_; }

private:

    // The raw depth image
    cv::Mat depth_raw_;

    // The depth image (converted to uint8_t)
    cv::Mat depth_;

    // The surface normals (converted to uint8_t)
    cv::Mat normal_;

    // The minimum and maximum depth values
    double depth_min_;
    double depth_max_;
};


#endif  // DEPTH_
