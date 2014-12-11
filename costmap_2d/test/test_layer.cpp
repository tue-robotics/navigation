#include <opencv2/highgui/highgui.hpp>
#include <costmap_2d/costmap_2d.h>

#include <costmap_2d/dilation_layer.h>

#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "costmap_2d_test");
    ros::NodeHandle bh;

    if (argc != 2)
    {
        std::cout << "Please provide image file" << std::endl;
        return 1;
    }

    // load image
    cv::Mat map_image = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);

    if (!map_image.data) {
        std::cerr << "Could not load image '" << argv[1] << "'." << std::endl;
        return 1;
    }

    // Show input
    cv::imshow("input", map_image);
    cv::waitKey(3);

    // get width and height
    unsigned int width = map_image.cols;
    unsigned int height = map_image.rows;

    costmap_2d::Costmap2D costmap(width, height, 0.025, 0, 0);
    for(unsigned int y = 0; y < height; ++y)
        for(unsigned int x = 0; x < width; ++x)
            costmap.setCost(x, y, map_image.at<unsigned char>(y, x));

    // Update
    costmap_2d::DilationLayer layer;
    layer.updateCosts(costmap, 0, 0, width, height);

    cv::Mat output_image = map_image.clone();
    for(unsigned int y = 0; y < height; ++y)
        for(unsigned int x = 0; x < width; ++x)
            output_image.at<unsigned char>(y, x) = costmap.getCost(x, y);

    cv::imshow("output", output_image);
    cv::waitKey();

    return 0;
}
