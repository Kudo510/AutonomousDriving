#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

class global_2dmap_postprocessing{
    ros::NodeHandle nh;
    ros::Subscriber projected_map_sub;
    ros::Publisher projected_map_pub;


public:
    global_2dmap_postprocessing() {
        projected_map_sub = nh.subscribe("projected_map", 1, &global_2dmap_postprocessing::projected_map_sub_cb, this);
        projected_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("projected_map_2", 1);
    }

    void projected_map_sub_cb(const nav_msgs::OccupancyGrid& input_grid) {
        int iterations = 3;
        nav_msgs::OccupancyGrid output_grid = input_grid;
        int width = input_grid.info.width;
        int height = input_grid.info.height;

        std::vector<int8_t>& data = output_grid.data;
        cv::Mat image(height, width, CV_8UC1, data.data());
        cv::Mat dilated_image;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1, 1));
        cv::Point anchor(-1, -1);
        cv::dilate(image, dilated_image, kernel, anchor, iterations);

        // Copy the dilated data back to the occupancy grid
        std::memcpy(data.data(), dilated_image.data, data.size());

        projected_map_pub.publish(output_grid);
    }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "global_2dmap_postprocessing");
    ROS_INFO_NAMED("global mapping (post processing 2d map)", "started!");
    global_2dmap_postprocessing n;
    ros::spin();
}
