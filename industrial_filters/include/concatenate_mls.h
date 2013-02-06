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
 * concantenate_mls.h
 *
 *  Created on: Jan 31, 2013
 *      Author: cgomez
 */

#ifndef CONCANTENATE_MLS_H_
#define CONCANTENATE_MLS_H_

#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

namespace industrial_filters
{

template <typename PointT>
class ConcantenateMLS : public pcl::Filter<PointT>
  {
  protected:
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

    const std::vector<PointCloudConstPtr>& getInputClouds() const
    {
      return input_clouds_;
    }

    void setInputClouds(const std::vector<PointCloudConstPtr>& inputClouds)
    {
      for (int a=0; a<inputClouds.size(); a++)//
        {
        input_clouds_.push_back(inputClouds.at(a));
        }
    }

  public:
    ConcantenateMLS();
    ~ConcantenateMLS();

  protected:
    void applyFilter (PointCloud &output);

    using pcl::Filter<PointT>::filter_name_;
    using pcl::PCLBase<PointT>::input_;

  private:
    std::vector<PointCloudConstPtr> input_clouds_;
    std::string filter_field_name_;
    float filter_limit_min_;
    float filter_limit_max_;
    PointCloudConstPtr cloud_;
    PointCloudPtr concat_cloud_;
    PointCloudConstPtr temp_cloud_;

    TreePtr tree_;
    pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls_;
    NormalCloudOut mls_points_;
    NormalCloudOutPtr normals_;
    float search_radius_;
  };

  template <>
  class ConcantenateMLS<sensor_msgs::PointCloud2> : public pcl::Filter<sensor_msgs::PointCloud2>
  {
  protected:
    typedef sensor_msgs::PointCloud2 PointCloud2;
    void applyFilter (PointCloud2 &output);

  public:
    const std::string& getFilterFieldName() const
    {
      return filter_field_name_;
    }

    void setFilterFieldName(const std::string& filterFieldName)
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

    bool getKeepOrganized() const
    {
      return keep_organized_;
    }

    void setKeepOrganized(bool keepOrganized)
    {
      keep_organized_ = keepOrganized;
    }

    bool getFilterLimitNegative() const
    {
      return filter_limit_negative_;
    }

    void getFilterLimitsNegative (bool &limit_negative)
    {
      limit_negative = filter_limit_negative_;
    }

    void setFilterLimitNegative(const bool filterLimitNegative)
    {
      filter_limit_negative_ = filterLimitNegative;
    }

  private:
    bool keep_organized_;
    std::string filter_field_name_;
    double filter_limit_min_;
    double filter_limit_max_;
    bool filter_limit_negative_;

  };
}


#endif /* CONCANTENATE_MLS_H_ */
