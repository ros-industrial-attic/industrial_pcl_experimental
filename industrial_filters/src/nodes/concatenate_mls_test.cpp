/*
 * Copyright 2013 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * concatenate_mls_test.cpp
 *
 *  Created on: Feb 1, 2013
 *      Author: cgomez
 */

#include "concatenate_mls.h"
#include "ros/ros.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *----------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "concatenate_average");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("avg_filtered_cloud", 100);


  // Declare variables that can be modified by launch file or command line.
  double search_radius_;
  int num_images_;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_nh_("~");
  private_nh_.param("mls_search_radius", search_radius_, double(0.003));
  private_nh_.param("num_images_concat", num_images_, int(10));
/*
  // Create a new NodeExample object.
  industrial_filters_nodelets::ConcantenateMLS *filter_example = new industrial_filters_nodelets::ConcantenateMLS();
  // Set up a dynamic reconfigure server.
  bool service=true; //true enables dynamic reconfigure service
  filter_example->child_init(n, service);
*/
  std::vector<sensor_msgs::PointCloud2::ConstPtr> input_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud  (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_cloud.at(0), *cloud);
  std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr > clouds;// (new std::vector<pcl::PointCloud<pcl::PointXYZ> >);

  industrial_filters::ConcatenateMLS<pcl::PointXYZ > concat_mls_filter;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices;
  sensor_msgs::PointCloud2 out_cloud;
  while (ros::ok())
  {
  std::string topic = n.resolveName("cloud_in");
  ROS_INFO("Cloud service called; waiting for a point_cloud2 on topic %s", topic.c_str());

  for (int k=0; k<num_images_; k++)
  {
    sensor_msgs::PointCloud2::ConstPtr recent_cloud =
    ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, n, ros::Duration(3.0));
    input_cloud.push_back(recent_cloud);
  }
  ROS_INFO_STREAM("Input Clouds gathered: "<<input_cloud.size() <<" PointCloud2's");
/*  clouds.push_back(cloud);*/
  for (int j=0; j<num_images_; j++)
  {
    pcl::fromROSMsg(*input_cloud.at(j), *cloud);
    clouds.push_back(cloud);
  }
  ROS_INFO_STREAM("Clouds set as input: "<<clouds.size() <<" PointClouds");

  concat_mls_filter.setInputCloud(cloud);
  concat_mls_filter.setInputClouds(clouds);
  concat_mls_filter.filter(*filtered_cloud);

  // Create a publish-able cloud.
  pcl::toROSMsg (*filtered_cloud, out_cloud);
  out_cloud.header.frame_id="/camera_depth_optical_frame";
  out_cloud.header.stamp=ros::Time::now();
  ROS_INFO_STREAM("Filtered cloud converted");
  // Publish the data
  pub.publish (out_cloud);
  ROS_INFO_STREAM("Filtered cloud published");
  }
}
