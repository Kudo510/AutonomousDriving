#pragma once

#include <unordered_map>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include "unity_stream_parser.h"
#include <cmath>

class TrueStateParser : public UnityStreamParser {

struct Quaternion {
    double w, x, y, z;
};

public:
  TrueStateParser() : nh_("~") { };

  virtual bool ParseMessage(const UnityHeader& header, 
                            TCPStreamReader& stream_reader,
                            double time_offset) override {
    float px, py, pz;
    float qw, qx, qy, qz;
    float qw2, qx2, qy2, qz2;
    float vx, vy, vz;
    float rx, ry, rz;
    
    Quaternion rotationQuaternion;
    
    rotationQuaternion.w = std::sqrt(2) / 2;
    rotationQuaternion.x = 0;
    rotationQuaternion.y = 0;
    rotationQuaternion.z = std::sqrt(2) / 2;

    px = stream_reader.ReadFloat();
    py = stream_reader.ReadFloat();
    pz = stream_reader.ReadFloat();
    
    qx = stream_reader.ReadFloat();
    qy = stream_reader.ReadFloat();
    qz = stream_reader.ReadFloat();
    qw = stream_reader.ReadFloat();
    
    qx2 = -qx;
    qy2 = -qz;
    qz2 = -qy;
    qw2 = qw;
    
    Quaternion rotatedQuaternion;
    rotatedQuaternion.w = qw2 * rotationQuaternion.w - qx2 * rotationQuaternion.x - qy2 * rotationQuaternion.y - qz2 * rotationQuaternion.z;
    rotatedQuaternion.x = qw2 * rotationQuaternion.x + qx2 * rotationQuaternion.w + qy2 * rotationQuaternion.z - qz2 * rotationQuaternion.y;
    rotatedQuaternion.y = qw2 * rotationQuaternion.y - qx2 * rotationQuaternion.z + qy2 * rotationQuaternion.w + qz2 * rotationQuaternion.x;
    rotatedQuaternion.z = qw2 * rotationQuaternion.z + qx2 * rotationQuaternion.y - qy2 * rotationQuaternion.x + qz2 * rotationQuaternion.w;
    
    double norm = std::sqrt(rotatedQuaternion.w * rotatedQuaternion.w + rotatedQuaternion.x * rotatedQuaternion.x + rotatedQuaternion.y * rotatedQuaternion.y + rotatedQuaternion.z * rotatedQuaternion.z);
    rotatedQuaternion.w /= norm;
    rotatedQuaternion.x /= norm;
    rotatedQuaternion.y /= norm;
    rotatedQuaternion.z /= norm;

    vx = stream_reader.ReadFloat();
    vy = stream_reader.ReadFloat();
    vz = stream_reader.ReadFloat();

    rx = stream_reader.ReadFloat();
    ry = stream_reader.ReadFloat();
    rz = stream_reader.ReadFloat();

    if(pose_publishers_.find(header.name) == pose_publishers_.end()) {
      pose_publishers_.insert(std::make_pair(header.name, nh_.advertise<geometry_msgs::PoseStamped>(header.name + "/pose", 10)));
      twist_publishers_.insert(std::make_pair(header.name, nh_.advertise<geometry_msgs::TwistStamped>(header.name + "/twist", 10)));
    }    

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = "world"; //"odom_nav";
    pose_msg.header.stamp = ros::Time(header.timestamp + time_offset);
    
    pose_msg.pose.position.x = px;
    pose_msg.pose.position.y = pz;
    pose_msg.pose.position.z = py;

    pose_msg.pose.orientation.x = rotatedQuaternion.x;
    pose_msg.pose.orientation.y = rotatedQuaternion.y;
    pose_msg.pose.orientation.z = rotatedQuaternion.z;
    pose_msg.pose.orientation.w = rotatedQuaternion.w;

    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.frame_id = "world"; //"odom_nav";
    twist_msg.header.stamp = pose_msg.header.stamp;
    
    twist_msg.twist.linear.x = vx;    
    twist_msg.twist.linear.y = vz;
    twist_msg.twist.linear.z = vy;

    twist_msg.twist.angular.x = rx;
    twist_msg.twist.angular.y = rz;
    twist_msg.twist.angular.z = ry;
       
    pose_publishers_[header.name].publish(pose_msg);
    twist_publishers_[header.name].publish(twist_msg);

    geometry_msgs::TransformStamped::Ptr tf(new geometry_msgs::TransformStamped);
    tf->header.stamp = pose_msg.header.stamp;
    tf->header.frame_id = "world";
    tf->child_frame_id = "true_body";

    tf->transform.translation.x = pose_msg.pose.position.x;
    tf->transform.translation.y = pose_msg.pose.position.y;
    tf->transform.translation.z = pose_msg.pose.position.z;

    tf->transform.rotation.x = pose_msg.pose.orientation.x;
    tf->transform.rotation.y = pose_msg.pose.orientation.y;
    tf->transform.rotation.z = pose_msg.pose.orientation.z;
    tf->transform.rotation.w = pose_msg.pose.orientation.w;

    tf_broadcaster.sendTransform(*tf);
  }

private:
  std::unordered_map<std::string, ros::Publisher> pose_publishers_;
  std::unordered_map<std::string, ros::Publisher> twist_publishers_;
  tf2_ros::TransformBroadcaster tf_broadcaster;
  ros::NodeHandle nh_;
};
