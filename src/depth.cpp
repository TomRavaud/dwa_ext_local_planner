#include <depth_image/depth.h>


Depth::Depth(cv::Mat depth,
             double depth_min,
             double depth_max) : depth_raw_(depth)
{   
    // Initialize the depth image
    depth_ = cv::Mat::zeros(depth_raw_.size(), CV_8UC1);

    // Initialize the normal image
    normal_ = cv::Mat::zeros(depth_raw_.size(), CV_8UC3);
    
    // Set the minimum and maximum depth values
    depth_min_ = depth_min;
    depth_max_ = depth_max;
}


void Depth::computeNormals(cv::Mat K,
                           double gradient_threshold)
{   
    // Get the focal lengths from the internal calibration matrix
    const double fx = K.at<double>(0, 0), fy = K.at<double>(1, 1);

    // Get the image dimensions
    const int height = depth_raw_.rows, width = depth_raw_.cols;

    // Go through each pixel in the depth image
    for(int v { 0 }; v < height; v++)
    {
        for(int u { 0 }; u < width; u++)
        {
            // Get the depth value at the current pixel
            float depth_value = depth_raw_.at<float>(v, u);

            // If the depth value is invalid, set the depth image pixel to
            // 0 (black)
            if (isnan(depth_value) || isinf(depth_value))
                depth_.at<uint8_t>(v, u) = 0;
            
            // Else convert the depth value to a grayscale value between 0
            // and 255
            else
                depth_.at<uint8_t>(v, u) =
                    255/(depth_max_-depth_min_)*(depth_value-depth_min_);
            
            // Define variables to store image gradients
            // u, v are the pixel coordinates in the image
            float dz_du, dz_dv;

            // Compute the gradient in the u direction
            if (u == 0)
                dz_du = depth_raw_.at<float>(v, u+1) - depth_value;
                
            else if (u == width-1)
                dz_du = depth_value - depth_raw_.at<float>(v, u-1);
            
            else
                dz_du = (depth_raw_.at<float>(v, u+1) -
                         depth_raw_.at<float>(v, u-1)) / 2.0;

            // Compute the gradient in the v direction
            if (v == 0)
                dz_dv = depth_raw_.at<float>(v+1, u) - depth_value;
            
            else if (v == height-1)
                dz_dv = depth_value - depth_raw_.at<float>(v-1, u);
            
            else
                dz_dv = (depth_raw_.at<float>(v+1, u) -
                         depth_raw_.at<float>(v-1, u)) / 2.0;

            // If the gradients are invalid, set the normal vector to the
            // default color
            if (isinf(dz_du) || isinf(dz_dv) || isnan(dz_du) || isnan(dz_dv))
            {
                normal_.at<cv::Vec3b>(v, u) = cv::Vec3b(255, 127, 127);
                continue;
            }

            // Derive the pinhole camera model
            // x, y, z are the coordinates in the camera coordinate system
            float du_dx = fx / depth_value;
            float dv_dy = fy / depth_value;

            // Apply the chain rule for the partial derivatives
            float dz_dx = dz_du * du_dx;
            float dz_dy = dz_dv * dv_dy;

            // Compute the gradient magnitude
            float gradient_magnitude = sqrt(dz_dx * dz_dx + dz_dy * dz_dy);

            // Check if the gradient magnitude is greater than the threshold
            // If so, use a default color for the normal vector
            if (gradient_magnitude > gradient_threshold)
            {
                normal_.at<cv::Vec3b>(v, u) = cv::Vec3b(255, 127, 127);
                continue;
            }

            // Compute the normal vector
            cv::Vec3f normal (-dz_dx, -dz_dy, 1.0f);

            // Normalize the normal vector
            normal = normalize(normal);

            // Scale the normal values to the range of 0-255
            int r = (normal[0] + 1.0) * 0.5 * 255;
            int g = (normal[1] + 1.0) * 0.5 * 255;
            int b = (normal[2] + 1.0) * 0.5 * 255;

            // Store the normal vector in the normal image
            // (OpenCV uses BGR color ordering instead of RGB)
            normal_.at<cv::Vec3b>(v, u) = cv::Vec3b(b, g, r);
        }
    }
}


void Depth::displayDepth() const
{
    cv::imshow("Depth", depth_);
}

void Depth::displayNormals() const
{
    cv::imshow("Normals", normal_);
}
