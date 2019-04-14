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

#include <pcl/point_types.h>

using namespace std;

Convert::Convert(ros::NodeHandle& nh, ros::NodeHandle& nh_local)
:nh_(nh), nh_local_(nh_local)
{
    this->readParams();
    rslidar_sub_ = nh.subscribe(rslidar_topic_, 1, &Convert::callback, this);
    velodyne_pub_ = nh.advertise<sensor_msgs::PointCloud2>(velodyne_topic_, 1);
#ifdef DEBUG
    for(int i = 0; i < 16; i++)
    {
        velo_pub_per_ring_[i] = nh.advertise<sensor_msgs::PointCloud2>("ring_" + to_string(i), 1);
    }
#endif
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
#ifdef DEBUG
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR> velo_pc_per_ring[16];
#endif // DEBUG
    for(int i = 0; i < 16; i++)
    {
        for(int j = 0; j < rs_pc.size() / 16; j++)
        {
            velodyne_pointcloud::PointXYZIR velo_point;
            pcl::PointXYZI& rs_point =  rs_pc[j + i * rs_pc.size() / 16];
            if(isnan(rs_point.x) || isnan(rs_point.y) || isnan(rs_point.y))
                continue;
            velo_point.x = rs_point.x;
            velo_point.y = rs_point.y;
            velo_point.z = rs_point.z;
            velo_point.intensity = rs_point.intensity;
            velo_point.ring = i < 8 ? i : 15 - (i - 8);
            velo_pc.push_back(velo_point);
#ifdef DEBUG
            // if(j < 100)
            velo_pc_per_ring[velo_point.ring].push_back(velo_point);
#endif // DEBUG
        }
    }
    sensor_msgs::PointCloud2 pub_cloud;
    pcl::toROSMsg(velo_pc, pub_cloud);
    pub_cloud.header.frame_id = "velodyne";
    pub_cloud.header.stamp = cloud->header.stamp;
    velodyne_pub_.publish(pub_cloud);
#ifdef DEBUG
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>  velo_pc_filter;
    for(int i = 0; i < 16; i++)
    {
        pcl::PointCloud<velodyne_pointcloud::PointXYZIR> velo_pc_per_ring_filter;
        for(int j = 1; j < velo_pc_per_ring[i].size()-1; j++)
        {
            velodyne_pointcloud::PointXYZIR& last_point = velo_pc_per_ring[i][j-1];
            double range_before = sqrt(last_point.z*last_point.z + 
            last_point.x*last_point.x + last_point.y*last_point.y);

            velodyne_pointcloud::PointXYZIR& cur_point = velo_pc_per_ring[i][j];
            double range_cur = sqrt(cur_point.z*cur_point.z + 
            cur_point.x*cur_point.x + cur_point.y*cur_point.y);

            velodyne_pointcloud::PointXYZIR& next_point = velo_pc_per_ring[i][j+1];
            double range_next = sqrt(next_point.z*next_point.z + 
            next_point.x*next_point.x + next_point.y*next_point.y);
            if(max(range_before - range_cur, range_next - range_cur) > 0.05)
            {
                velo_pc_per_ring_filter.push_back(velo_pc_per_ring[i][j]);
                // ROS_INFO("IGETPOINT %d", i);
            }
        }
        velo_pc_per_ring[i] = velo_pc_per_ring_filter;
        velo_pc_filter += velo_pc_per_ring_filter;
        // sensor_msgs::PointCloud2 pub_cloud_per_ring;
        // pcl::toROSMsg(velo_pc_per_ring[i], pub_cloud_per_ring);
        // pub_cloud_per_ring.header.frame_id = "velodyne";
        // pub_cloud_per_ring.header.stamp = cloud->header.stamp;
        // velo_pub_per_ring_[i].publish(pub_cloud_per_ring);
    }
    sensor_msgs::PointCloud2 pub_cloud_per_ring;
    pcl::toROSMsg(velo_pc_filter, pub_cloud_per_ring);
    pub_cloud_per_ring.header.frame_id = "velodyne";
    pub_cloud_per_ring.header.stamp = cloud->header.stamp;
    velo_pub_per_ring_[0].publish(pub_cloud_per_ring);
#endif // DEBUG
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "rslidar_to_velodyne");
    ros::NodeHandle nh, nh_local("rslidar_to_velodyne");
    Convert cc(nh, nh_local);
    ros::spin();
}