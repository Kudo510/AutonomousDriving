#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_listener.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <unordered_set>

class point_cloud_filtering{
    ros::NodeHandle nh;
    ros::Subscriber point_cloud_sub;
    ros::Publisher transformed_point_cloud_pub;

public:
    point_cloud_filtering() {
        point_cloud_sub = nh.subscribe("points_cloud", 10, &point_cloud_filtering::transform_point_cloud_cb, this);
        transformed_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("transformed_cloud", 10);
    }

    void transform_point_cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
        // Convert msg into a point-cloud-object
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // Rotate point cloud points -90 degrees around X-axis
        // This is to make it possible for the octomal to correctly generate the 2D map
        Eigen::Matrix4f transformationX = Eigen::Matrix4f::Identity();
        float angleX = -M_PI / 2.0;
        transformationX(1, 1) = cos(angleX);
        transformationX(1, 2) = -sin(angleX);
        transformationX(2, 1) = sin(angleX);
        transformationX(2, 2) = cos(angleX);
        pcl::transformPointCloud(*cloud, *cloud, transformationX);

        // Filtered cloud to reduce the noise associated to measurements
        //   - discard all points within a 1m radio because they might be too unstable
        //   - discard points with an infinite or nan values
        pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const pcl::PointXYZ& point : cloud->points) {
            // If point is outside the specified radius, add it to the filtered cloud
            if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)){
                const bool is_outside_1m_radio = std::sqrt(point.x * point.x + point.y * point.y) > 1.0;
                const bool is_within_depth = point.z > -5 && point.z < 10;
                if (is_within_depth && is_outside_1m_radio) {
                    filteredCloud->points.push_back(point);
                }
            }
        }
        // Update the header information of the filtered cloud
        filteredCloud->header = cloud->header;
        filteredCloud->width = filteredCloud->points.size();
        filteredCloud->height = 1;

        //ROS_INFO("cloud->size(): %d", cloud->size());
        //ROS_INFO("cloud->width: %d", cloud->width);
        //ROS_INFO("cloud->height: %d", cloud->height);
        //ROS_INFO("filteredCloud->size(): %d", filteredCloud->points.size());

        // Publish the new cloud (transformed and filtered)
        sensor_msgs::PointCloud2 transformed_cloud_msg;
        pcl::toROSMsg(*filteredCloud, transformed_cloud_msg);
        transformed_cloud_msg.header = msg->header;
        transformed_point_cloud_pub.publish(transformed_cloud_msg);
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "point_cloud_filtering");
    ROS_INFO_NAMED("global mapping", "started!");
    point_cloud_filtering n;
    ros::spin();
}
