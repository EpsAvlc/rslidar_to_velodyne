/*
 * Created on Thu Apr 11 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

class Convert
{
public:
    Convert(ros::NodeHandle& nh, ros::NodeHandle& nh_local);
private:
    void readParams();
    void callback(const sensor_msgs::PointCloud2ConstPtr& cloud);
    /* ROS params*/
    ros::NodeHandle nh_, nh_local_;
    ros::Subscriber rslidar_sub_;
    ros::Publisher velodyne_pub_;
    /* class variable */
    std::string rslidar_topic_;
    std::string velodyne_topic_;
};
