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
 *
 * concatenate_mls.cpp
 *
 *  Created on: Jan 31, 2013
 *      Author: cgomez
 */

#include "concatenate_mls.h"
#include <pcl/point_types.h>
#include <pcl/surface/processing.h>
#include <pcl/surface/mls.h>
#include "pcl/impl/instantiate.hpp"
#include "pcl/point_types.h"
#include "ros/ros.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
industrial_pcl_filters::ConcatenateMLS<PointT>::ConcatenateMLS():
    filter_limit_min_(FLT_MIN),
    filter_limit_max_(FLT_MAX),
    filter_field_name_(""),
    search_radius_(0.003)
    {
      //Constructor
      filter_name_="ConcatenateMLS";
    }
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
industrial_pcl_filters::ConcatenateMLS<PointT>::~ConcatenateMLS()
{
  //Destructor
  input_clouds_.clear();
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void industrial_pcl_filters::ConcatenateMLS<PointT>::applyFilter(PointCloud &output)
{

  ROS_INFO_STREAM("Starting custom filtering");
  int clouds_size = input_clouds_.size();
  ROS_INFO_STREAM("Number of clouds to concatenate "<< clouds_size);
  //If you put your class definition in a .h file, you must put the initialization in the .cc file, not the .h file.
  temp_concat_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (clouds_size>0)
  {
    ROS_INFO_STREAM("More than one cloud, start concatenating");
    //for (iter_=input_clouds_.begin(); iter_!=input_clouds_.end(); ++iter_)
    for (int i=0; i<clouds_size; i++ )
    {
      //ROS_INFO_STREAM("Beginning to concatenate "<< i <<"th cloud");
      temp_cloud_=input_clouds_.at(i);
      ROS_INFO_STREAM("Temp cloud size "<< temp_cloud_.points.size());
      *temp_concat_cloud_+= temp_cloud_;
      ROS_INFO_STREAM("Concat cloud size "<< temp_concat_cloud_->size());
    }
  }
  else
  {
    ROS_ERROR_STREAM("No clouds passed into concatenate/MLS function");
  }

  ROS_INFO_STREAM("cloud after concat has "<<temp_concat_cloud_->size() <<" points");

  cloud_=temp_concat_cloud_;
  mls_.setOutputNormals(normals_);
  mls_.setInputCloud(cloud_);
  //mls_.setInputCloud(input_);
  mls_.setPolynomialFit (true);
  mls_.setSearchMethod (tree_);
  mls_.setSearchRadius (search_radius_);

  mls_.reconstruct(output);

  ROS_INFO_STREAM("cloud after MLS has "<<output.size() <<" points");

  input_clouds_.clear();

}
//////////////////////////////////////////////////////////////////////////////////////////////
void industrial_pcl_filters::ConcatenateMLS<sensor_msgs::PointCloud2>::applyFilter(sensor_msgs::PointCloud2 &output)
{
  ROS_INFO_STREAM("Starting custom filtering ...");
  ROS_INFO("ConcatenateMLS node called; waiting for a point_cloud2 on topic %s", topic_.c_str());
  concat_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  ROS_INFO_STREAM("No. input clouds: "<<num_images_);
  for (int k=0; k<num_images_; k++)
  {
    recent_cloud_ = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic_, ros::Duration(3.0));
    //input_clouds_.push_back(recent_cloud_);
    pcl::fromROSMsg(*recent_cloud_, cloud_);
    ROS_INFO_STREAM("Beginning to concatenate "<< k <<" cloud");
    *concat_cloud_+=cloud_;
    //clouds_.push_back(cloud_);
  }
/*  ROS_INFO_STREAM("Input Clouds gathered: "<<input_clouds_.size() <<" PointCloud2's");

  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud  (new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::fromROSMsg(*input_clouds_.at(0), cloud_);
  //std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr > clouds;
  for (int j=0; j<num_images_; j++)
    {
      pcl::fromROSMsg(*input_clouds_.at(j), cloud_);
      clouds_.push_back(cloud_);
    }
  ROS_INFO_STREAM("Clouds set as input: "<<clouds_.size() <<" PointClouds");
  if (clouds_.size()>0)
    {
      ROS_INFO_STREAM("More than one cloud, start concatenating");
      for (int i=0; i<clouds_.size(); i++ )
      {
        ROS_INFO_STREAM("Beginning to concatenate "<< i <<"th cloud");
        temp_cloud_=clouds_.at(i);
        *concat_cloud_+= temp_cloud_;
      }
    }
  else
  {
    ROS_ERROR_STREAM("No clouds passed into concatenate/MLS function");
  }
*/
  ROS_INFO_STREAM("cloud after concat has "<<concat_cloud_->size() <<" points");

  mls_.setOutputNormals(normals_);
  mls_.setInputCloud(concat_cloud_);
  mls_.setPolynomialFit (true);
  mls_.setSearchMethod (tree_);
  mls_.setSearchRadius (search_radius_);

  mls_.reconstruct(output_pc_);
  ROS_INFO_STREAM("cloud after MLS has "<<output_pc_.size() <<" points");

  //pcl::toROSMsg(*concat_cloud_, output);
  pcl::toROSMsg(output_pc_, output);
  output.header.frame_id="/camera_rgb_optical_frame";
  output.header.stamp=ros::Time::now();
  ROS_INFO_STREAM("Filtered cloud converted");

  concat_cloud_.reset();
}
//////////////////////////////////////////////////////////////////////////////////////////////
industrial_pcl_filters::ConcatenateMLS<sensor_msgs::PointCloud2>::ConcatenateMLS():
keep_organized_(false),
user_filter_value_ (std::numeric_limits<float>::quiet_NaN ()),
filter_field_name_(""),
filter_limit_min_(FLT_MIN),
filter_limit_max_(FLT_MAX),
num_images_(3),
search_radius_(0.003)
  {
    filter_name_="ConcatenateMLS";
  }
//////////////////////////////////////////////////////////////////////////////////////////////
industrial_pcl_filters::ConcatenateMLS<sensor_msgs::PointCloud2>::~ConcatenateMLS()
  {
    //input_clouds_.clear();
  }

//Partial list of point types
//template class industrial_pcl_filters::ConcatenateMLS<pcl::PointXYZ>;
//template class industrial_pcl_filters::ConcatenateMLS<pcl::PointXYZI>;
template class industrial_pcl_filters::ConcatenateMLS<pcl::PointXYZRGB>;
/*template class industrial_pcl_filters::ConcatenateMLS<pcl::PointXYZRGBA>;
template class industrial_pcl_filters::ConcatenateMLS<pcl::PointXY>;
template class industrial_pcl_filters::ConcatenateMLS<pcl::InterestPoint>;
template class industrial_pcl_filters::ConcatenateMLS<pcl::Normal>;
template class industrial_pcl_filters::ConcatenateMLS<pcl::PointNormal>;
template class industrial_pcl_filters::ConcatenateMLS<pcl::PointXYZRGBNormal>;*/
//The macro PCL_INSTANTIATE does nothing else but go over a given list of types and creates an explicit instantiation for each
//#define PCL_INSTANTIATE(ConcatenateMLS, PCL_XYZ_POINT_TYPES);
