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

#include "industrial_pcl_filters/concatenate_mls.h"
#include <ros/ros.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/bilateral.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h> 
//#include <tabletop_object_detector/TabletopSegmentation.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

/*------------------------------------------------------------------------------
 * main()
 * Main function to set up ROS node.
 *----------------------------------------------------------------------------*/

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "concatenate_average");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("ur5_arm/avg_filtered_cloud", 100);
  //ros::ServiceClient seg_srv_ = n.serviceClient<tabletop_object_detector::TabletopSegmentation>("ur5_arm/tabletop_segmentation", true);
  ROS_INFO_STREAM("Initialized concatenating and MLS averaging node");

  // Declare variables that can be modified by launch file or command line.
  double search_radius_;
  int num_images_;
  int mean_k_;
  double mul_thresh_;
  bool use_bilateral_;
  double sigma_s_;
  double sigma_r_;
  double x_filter_min_;
  double x_filter_max_;
  double y_filter_min_;
  double y_filter_max_;
  std::string world_frame_;

  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ROS_INFO_STREAM("Loading private parameters (search radius and number of point clouds to concatenate)");
  ros::NodeHandle private_nh_("~");
  private_nh_.param("mls_search_radius", search_radius_, double(0.003));
  private_nh_.param("num_images_concat", num_images_, int(10));
  private_nh_.param("sor_mean", mean_k_, int(10));
  private_nh_.param("sor_thresh", mul_thresh_, double(1.0));
  private_nh_.param("use_bilateral", use_bilateral_, bool(false));
  private_nh_.param("sigma_s", sigma_s_, double(1.0));
  private_nh_.param("sigma_r", sigma_r_, double(1.0));
  private_nh_.param("x_filter_min", x_filter_min_, double(-1.0));
  private_nh_.param("x_filter_max", x_filter_max_, double(1.0));
  private_nh_.param("y_filter_min", y_filter_min_, double(-1.0));
  private_nh_.param("y_filter_max", y_filter_max_, double(1.0));
  private_nh_.getParam("world_frame", world_frame_);

  //Declare all types of clouds used
  //sensor_msgs::PointCloud2::ConstPtr recent_cloud (new sensor_msgs::PointCloud2);
  std::vector<sensor_msgs::PointCloud2::ConstPtr> input_clouds;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB> > clouds;// (new std::vector<pcl::PointCloud<pcl::PointXYZRGB> >);
  pcl::PointCloud<pcl::PointXYZRGB> cloud;//  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB> xf_cloud;//  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB> yf_cloud;//  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr Cloud  (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr sor_filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr bilateral_pre_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr bilateral_filtered_cloud (new pcl::PointCloud<pcl::PointXYZI>);
  sensor_msgs::PointCloud2 out_cloud;
  sensor_msgs::PointCloud cluster;

  //Make instance of concat_mls class
  industrial_pcl_filters::ConcatenateMLS<pcl::PointXYZRGB > concat_mls_filter;
  //Segmentation
/*  tabletop_object_detector::TabletopSegmentation segmentation_srv;
  //while (ros::ok())
  //{
  std::string topic = n.resolveName("cloud_in");
  ROS_INFO("Cloud service called; waiting for a point_cloud2 on topic %s", topic.c_str());
  sensor_msgs::PointCloud clustervector;
  clustervector=segmentation_srv.response.clusters[0];
  ROS_INFO_STREAM("No. input clouds to gather: "<<num_images_);
  for (int k=0; k<num_images_; k++)
  {
    recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic, n, ros::Duration(3.0));
    //input_cloud.push_back(recent_cloud);
    pcl::fromROSMsg(*recent_cloud, *cloud);
    clouds.push_back(cloud);
  }
  ROS_INFO_STREAM("Input Clouds gathered: "<<input_cloud.size() <<" PointCloud2's");

  ROS_INFO_STREAM("No. clouds to convert: "<<num_images_);
  for (int j=0; j<num_images_; j++)
  {
    pcl::fromROSMsg(*input_cloud.at(j), *cloud);
    clouds.push_back(cloud);

        if (!seg_srv_.call(segmentation_srv))
      {
        ROS_ERROR("Call to segmentation service failed");
      }
    if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
      {
        ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);
      }

    ROS_INFO_STREAM("Segmentation service succeeded. Detected " <<segmentation_srv.response.clusters.size()
             << " clusters of " << segmentation_srv.response.clusters[0].points.size() << " size");

    cluster=segmentation_srv.response.clusters[0];
    ROS_INFO_STREAM("Cluster size: "<<cluster.points.size());
    sensor_msgs::convertPointCloudToPointCloud2(cluster, *recent_cloud);
    recent_cloud->header.frame_id="/ur5_arm_kinect_rgb_optical_frame";
    recent_cloud->header.stamp=ros::Time::now();
    ROS_INFO_STREAM("Recent_cloud size: "<<recent_cloud->data.size());
    input_clouds.push_back(recent_cloud);
    pcl::fromROSMsg(*recent_cloud, cloud);
    ROS_INFO_STREAM("Cloud size: "<<cloud.points.size());
  }*/

  while (ros::ok())
 {
  for (int j=0; j<num_images_; j++)
  {

        cluster.points.clear();
        //recent_cloud.reset();
        //recent_cloud->data.clear();
        //cloud->points.clear();

    cloud.points.clear();
    std::string topic = n.resolveName("cloud_in");
    ROS_INFO("Cloud service called; waiting for a point_cloud2 on topic %s", topic.c_str());
    sensor_msgs::PointCloud2::ConstPtr recent_cloud =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic);

      sensor_msgs::PointCloud old_cloud;
      sensor_msgs::PointCloud2 transformed_cloud;
      std::string processing_frame = world_frame_;

      pcl::fromROSMsg(*recent_cloud, cloud);
      //cloud = obj_points;
      tf::TransformListener tf_listener;
      tf::StampedTransform sceneTf; sceneTf.setIdentity();
      //std::string clusterFrameId = "/kinect_rgb_optical_frame";
      std::string clusterFrameId = recent_cloud->header.frame_id;
      ROS_INFO_STREAM("Cloud passed to recognition service with frame id: "<<clusterFrameId);
      try
      {
        tf_listener.waitForTransform(processing_frame, clusterFrameId, ros::Time::now(), ros::Duration(1.0));
        tf_listener.lookupTransform(processing_frame ,clusterFrameId, ros::Time(0),sceneTf);
        ROS_INFO_STREAM("Successfully received sceneTF on tf_listener for "<< clusterFrameId << " to "<< processing_frame);
      }
      catch(tf::TransformException ex)
      {
              ROS_ERROR("%s",std::string("Failed to resolve transform from " +
                              processing_frame + " to " + clusterFrameId + " \n\t\t" + " tf error msg: " +  ex.what()).c_str());
              ROS_WARN("%s",std::string("Will use Identity as cluster transform").c_str());
              sceneTf.setData(tf::Transform::getIdentity());
              return false;
      }

      pcl::PointCloud<pcl::PointXYZRGB> transformed_scene;
      Eigen::Affine3d tfEigen_scene;
      tf::transformTFToEigen(sceneTf,tfEigen_scene);//clusterTf from ClusterFrame to WorldFrame
      pcl::transformPointCloud(cloud,transformed_scene,Eigen::Affine3f(tfEigen_scene));



    //pcl::fromROSMsg(*recent_cloud, cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(transformed_scene));
    pcl::PassThrough<pcl::PointXYZRGB> pass_x;
    pass_x.setInputCloud(cloud_ptr);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_filter_min_, x_filter_max_);
    pass_x.filter(xf_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr f_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>(xf_cloud));
    pcl::PassThrough<pcl::PointXYZRGB> pass_y;
    pass_y.setInputCloud(f_cloud_ptr);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_filter_min_, y_filter_max_);
    pass_y.filter(yf_cloud);

    ROS_INFO_STREAM("Passthrough filtered cloud size: "<<yf_cloud.points.size());
    clouds.push_back(yf_cloud);

    //ROS_INFO_STREAM("input_clouds at "<< j <<" has "<< input_clouds.at(j)->data.size() <<" points");

  }

  ROS_INFO_STREAM("Clouds set as input: "<<clouds.size() <<" PointClouds");

  //Use concat mls filter
  concat_mls_filter.setInputCloud(Cloud);
  concat_mls_filter.setInputClouds(clouds);
  concat_mls_filter.filter(*mls_filtered_cloud);
  input_clouds.clear();
  clouds.clear();


  //Use Bilateral Filter
  //Bilateral filter is instantiated in pcl_flters only for pcl::PointXYZI and pcl::PointXYZINormal
  ROS_INFO_STREAM("Beginning Bilateral Filtering");
  pcl::copyPointCloud(*mls_filtered_cloud, *bilateral_pre_cloud);
  if(use_bilateral_)
  {
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZI>);
    ROS_INFO_STREAM("Converted cloud to XYZI, and set up tree");
    pcl::BilateralFilter<pcl::PointXYZI> bf;
    bf.setInputCloud (bilateral_pre_cloud);
    //bf.setSearchMethod (tree);
    bf.setHalfSize (sigma_s_);
    bf.setStdDev (sigma_r_);
    bf.filter (*bilateral_filtered_cloud);
    ROS_INFO_STREAM("Bilateral Filtering complete");
  }

  //Remove outliers
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  if(use_bilateral_)
  {
    sor.setInputCloud(bilateral_filtered_cloud);
  }
  else
  {
    sor.setInputCloud(bilateral_pre_cloud);
  }
  sor.setMeanK(mean_k_);
  sor.setStddevMulThresh(mul_thresh_);
  sor.filter(*sor_filtered_cloud);


  // Create a publish-able cloud.
  pcl::toROSMsg (*sor_filtered_cloud, out_cloud);
  out_cloud.header.frame_id=world_frame_;
  out_cloud.header.stamp=ros::Time::now();
  ROS_INFO_STREAM("Filtered cloud converted to ROS msg");
  // Publish the data
  pub.publish (out_cloud);
  ROS_INFO_STREAM("Filtered cloud published with "<<out_cloud.width<<" points");
/*
  //Create .pcd file to store and compare later
  pcl::PCDWriter writer;
  std::stringstream fileName_ss, fileName2;
  fileName_ss << "pcd_files/" << num_images_ << "_" << search_radius_*1000 << "_intensity.pcd";
  writer.write(fileName_ss.str(), *sor_filtered_cloud);
  ROS_INFO_STREAM("Saved " << sor_filtered_cloud->points.size() << " data points to " << fileName_ss.str());
  fileName2 << "pcd_files/1.pcd";
  writer.write(fileName2.str(), cloud);
  ROS_INFO_STREAM("Saved " << cloud.points.size() << " data points to " << fileName2.str());
  */
  }
}
