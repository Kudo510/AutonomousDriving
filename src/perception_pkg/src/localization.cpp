#include "ros/ros.h"

#include <std_msgs/String.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h>
#include <iostream>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <tf/transform_listener.h>

class Localization{
    private:
        ros::NodeHandle nh;
        tf::StampedTransform  base_link_to_world_transform;

        float current_x , current_y, current_z;
        float current_rot_x , current_rot_y, current_rot_z, current_rot_w;
        float roll_ , pitch_ , yaw_ ;


    public:
        Localization(){
            // Setting the map reference frame with respect to the world
            get_base_link_to_world_transform();
        }

        void get_base_link_to_world_transform(){
            tf::TransformListener tf_listener;
            try {
                tf_listener.waitForTransform("true_body", "world", ros::Time(0), ros::Duration(5.0));
                tf_listener.lookupTransform("true_body", "world", ros::Time(0), base_link_to_world_transform);

                tf::Transform rotation_transform;
                rotation_transform.setRotation(tf::Quaternion(0, 0, -M_PI / 2.0));

                // Apply the rotation to the original transform
                tf::Transform rotated_transform = rotation_transform * base_link_to_world_transform;

                base_link_to_world_transform.setRotation(rotated_transform.getRotation());
                base_link_to_world_transform.setOrigin(rotated_transform.getOrigin());

                static tf::TransformBroadcaster tf_broadcaster;
                tf_broadcaster.sendTransform(tf::StampedTransform(base_link_to_world_transform, ros::Time::now(), "base_link", "world"));

            } catch (tf::TransformException& ex) {
                ROS_ERROR("Failed to look up the base_link_to_world_transform %s", ex.what());
            }
        }

        void update_transforms()
        {
            //ros::Rate rate(1.0);

            while (ros::ok())
            {
                try
                {
                    // Re-publishing the map fixed reference frame (every time base_link changes)
                    get_base_link_to_world_transform();
                }
                catch (tf::TransformException& ex)
                {
                    ROS_ERROR("Failed to lookup transform: %s", ex.what());
                }

                //rate.sleep();
            }
        }

};

int main(int argc, char** argv){
    ros::init(argc, argv, "localization_node");
    ROS_INFO("localization_node started!");
    Localization localization;

    localization.update_transforms();

    ros::spin();
}