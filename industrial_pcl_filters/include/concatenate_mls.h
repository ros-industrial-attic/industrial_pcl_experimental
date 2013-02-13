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
 * concatenate_mls.h
 *
 *  Created on: Jan 31, 2013
 *      Author: cgomez
 */

#ifndef CONCANTENATE_MLS_H_
#define CONCANTENATE_MLS_H_

#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

namespace industrial_pcl_filters
{

template <typename PointT>
class ConcatenateMLS : public pcl::Filter<PointT>
  {

    typedef typename pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef pcl::PointCloud<pcl::PointNormal> NormalCloudOut;
    typedef typename NormalCloudOut::Ptr NormalCloudOutPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr TreePtr;

  public:
    const std::string& getFilterFieldName() const
    {
      return filter_field_name_;
    }

    void setFilterFieldName(const std::string& filterFieldName)
    {
      filter_field_name_ = filterFieldName;
    }

    inline void getFilterLimits (float &limit_min, float &limit_max)
    {
      limit_max= filter_limit_max_;
      limit_min= filter_limit_min_;
    }

    inline void setFilterLimits (float &limit_min, float &limit_max)
    {
      filter_limit_max_ = limit_max;
      filter_limit_min_ = limit_min;
    }

    const std::vector<PointCloud>& getInputClouds() const
    {
      return input_clouds_;
    }

    void setInputClouds(const std::vector<PointCloud>& inputClouds)
    {
      for (int a=0; a<inputClouds.size(); a++)//
        {
        input_clouds_.push_back(inputClouds.at(a));
        }
    }

  public:
    ConcatenateMLS();
    ~ConcatenateMLS();

  protected:
    void applyFilter (PointCloud &output);

    using pcl::Filter<PointT>::filter_name_;
    using pcl::PCLBase<PointT>::input_;

  private:
    std::vector<PointCloud> input_clouds_;
    std::string filter_field_name_;
    float filter_limit_min_;
    float filter_limit_max_;
    PointCloudConstPtr cloud_;
    PointCloudPtr concat_cloud_;
    PointCloud temp_cloud_;

    TreePtr tree_;
    pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls_;
    NormalCloudOut mls_points_;
    NormalCloudOutPtr normals_;
    float search_radius_;
  };
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  template <>
  class ConcatenateMLS<sensor_msgs::PointCloud2> : public pcl::Filter<sensor_msgs::PointCloud2>
  {
  //typedef sensor_msgs::PointCloud2 PointCloud2;
  //typedef PointCloud2::Ptr PointCloud2Ptr;
  //typedef PointCloud2::ConstPtr PointCloud2ConstPtr;
  typedef pcl::PointCloud<pcl::PointNormal> NormalCloudOut;
  typedef typename NormalCloudOut::Ptr NormalCloudOutPtr;
  typedef typename pcl::search::KdTree<pcl::PointXYZRGB>::Ptr TreePtr;
  protected:

  public:
    ConcatenateMLS ();
    ~ConcatenateMLS();
    inline const std::string& getFilterFieldName() const
    {
      return filter_field_name_;
    }

    inline void setFilterFieldName(const std::string& filterFieldName)
    {
      filter_field_name_ = filterFieldName;
    }

    inline void getFilterLimits (double &limit_min, double &limit_max)
    {
      limit_max= filter_limit_max_;
      limit_min= filter_limit_min_;
    }

    inline void setFilterLimits (const double &limit_min, const double &limit_max)
    {
      filter_limit_max_ = limit_max;
      filter_limit_min_ = limit_min;
    }

    inline bool getKeepOrganized() const
    {
      return keep_organized_;
    }

    inline void setKeepOrganized(bool keepOrganized)
    {
      keep_organized_ = keepOrganized;
    }

    inline bool getFilterLimitNegative() const
    {
      return filter_limit_negative_;
    }

    inline void getFilterLimitsNegative (bool &limit_negative)
    {
      limit_negative = filter_limit_negative_;
    }

    inline void setFilterLimitNegative(const bool filterLimitNegative)
    {
      filter_limit_negative_ = filterLimitNegative;
    }

    float getUserFilterValue() const
    {
      return user_filter_value_;
    }

    int getNumImages() const
    {
      return num_images_;
    }

    void setNumImages(int numImages)
    {
      num_images_ = numImages;
    }

    const std::string& getTopic() const
    {
      return topic_;
    }

    void setTopic(const std::string& topic)
    {
      topic_ = topic;
    }

    float getSearchRadius() const
    {
      return search_radius_;
    }

    void setSearchRadius(float searchRadius)
    {
      search_radius_ = searchRadius;
    }

  protected:
    void applyFilter (sensor_msgs::PointCloud2 &output);

  private:
    bool keep_organized_;
    std::string filter_field_name_;
    double filter_limit_min_;
    double filter_limit_max_;
    bool filter_limit_negative_;
    float user_filter_value_;
    std::vector<sensor_msgs::PointCloud2::ConstPtr> input_clouds_;
    int num_images_;
    std::string topic_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr concat_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr temp_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB> output_pc_;
    TreePtr tree_;
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls_;
    NormalCloudOut mls_points_;
    NormalCloudOutPtr normals_;
    float search_radius_;
    using pcl::Filter<sensor_msgs::PointCloud2>::removed_indices_;
    using pcl::Filter<sensor_msgs::PointCloud2>::extract_removed_indices_;

  };
}

#endif /* CONCATENATE_MLS_H_ */
