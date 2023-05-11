#include <depth_image/depth.h>


Depth::Depth(cv::Mat depth_image) : depth_image_(depth_image) {}

cv::Mat Depth::computeNormals()
{   
    // Convert the depth image to CV_32FC1 if it is not already
    if(depth_image_.type() != CV_32FC1)
    {
        std::cout << "Converting depth image to CV_32FC1" << std::endl;
        depth_image_.convertTo(depth_image_, CV_32FC1);
    }

    // Compute the normals
    cv::Mat normals(depth_image_.size(), CV_32FC3);

    for(int x = 1; x < depth_image_.rows-1; ++x)
    {
        for(int y = 1; y < depth_image_.cols-1; ++y)
        {   
            // Compute the gradient in the x and y directions
            float grad_x = (depth_image_.at<float>(x+1, y) - depth_image_.at<float>(x-1, y)) / 2.0;
            float grad_y = (depth_image_.at<float>(x, y+1) - depth_image_.at<float>(x, y-1)) / 2.0;

            // Compute the normal vector
            cv::Vec3f d(-grad_x, -grad_y, 1.0f);

            // Normalize the normal vector
            cv::Vec3f n = normalize(d);

            // Store the normal vector
            normals.at<cv::Vec3f>(x, y) = n;
        }
    }
    return normals;
}


cv::Mat convert_range(
    cv::Mat image,
    float min1,
    float max1,
    float min2,
    float max2)
{
    // Create the output image
    cv::Mat output_image(image.size(), CV_32FC1);

    // Compute the linear transformation that maps [min1, max1] to [min2, max2]
    float A = (max2 - min2) / (max1 - min1);
    float B = (min2 * max1 - min1 * max2) / (max1 - min1);

    for (int i = 0; i < image.rows; i++)
    {
        for (int j = 0; j < image.cols; j++)
        {
            // Apply the linear transformation
            float x1 = image.at<float>(i, j);
            float x2 = A * x1 + B;

            output_image.at<float>(i, j) = x2;
        }
    }
    return output_image;
}

void displayImage(cv::Mat image)
{
    double min_val, max_val;
    cv::Point min_loc, max_loc;

    // Find the minimum and maximum values in the image
    minMaxLoc(image, &min_val, &max_val, &min_loc, &max_loc);

    // Convert the image to a range of 0-255
    cv::Mat image_to_display = convert_range(image, 0.7, 7, 0, 255);

    // Convert the image to 8-bit
    image.convertTo(image, CV_8U);

    // Display the image
    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE);
    cv::imshow("Image", image_to_display);
}
