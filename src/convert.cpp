/*
 * Created on Thu Apr 11 2019
 *
 * Copyright (c) 2019 HITSZ-NRSL
 * All rights reserved
 *
 * Author: EpsAvlc
 */

#include "rslidar_to_velodyne/convert.h"

#include <iostream>
#include <string>

#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <pcl/point_types.h>

using namespace std;

Convert::Convert(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
:nh_(nh), nh_local_(nh_local)
{
    this->readParams();
    // velodyne_sub_ = nh.subscribe(velodyne_topic_, 1, &Convert::callback, this);
    rslidar_sub_ = nh.subscribe(rslidar_topic_, 1, &Convert::callback, this);
    velodyne_pub_ = nh.advertise<sensor_msgs::PointCloud2>(velodyne_topic_, 1);
}

void Convert::readParams()
{
    nh_local_.param<string>("rslidar_topic", rslidar_topic_, "rslidar_points");
    nh_local_.param<string>("velodyne_topic", velodyne_topic_, "converted_velodyne_points");
}

void Convert::callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    pcl::PointCloud<pcl::PointXYZI> rs_pc;
    pcl::fromROSMsg(*cloud, rs_pc);
    rs_pc.height = 16;
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> velo_pc;
    for(int i = 0; i < 16; i++)
    {
        for(int j = 0; j < rs_pc.size() / 16; j++)
        {
            velodyne_pointcloud::PointXYZIR velo_point;
            pcl::PointXYZI& rs_point =  rs_pc[j + i * rs_pc.size() / 16];
            velo_point.x = rs_point.x;
            velo_point.y = rs_point.y;
            velo_point.z = rs_point.z;
            velo_point.intensity = rs_point.intensity;
            velo_point.ring = i < 8 ? i : 16 - (i - 8);
            velo_pc.push_back(velo_point);
        }
    }
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(velo_pc, pub_cloud);
    pub_cloud.header.frame_id = "velodyne";
    pub_cloud.header.stamp = cloud->header.stamp;
    velodyne_pub_.publish(pub_cloud);
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "rslidar_to_velodyne");
    ros::NodeHandle nh, nh_local("rslidar_to_velodyne");
    Convert cc(nh, nh_local);
    ros::spin();
}