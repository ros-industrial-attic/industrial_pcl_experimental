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
/**
 * \brief The ConcatenateMLS filter adds clouds to each other and then applies an MLS filter.
 *
 * It uses the base pcl::filter class for standard input, output, and filter methods.
 * It sets up a few new parameters used in the nodelet class for dynamic reconfiguration.
 */
template <typename PointT>
class ConcatenateMLS : public pcl::Filter<PointT>
  {

    typedef typename pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef pcl::PointCloud<pcl::PointNormal> NormalCloudOut;
    typedef typename NormalCloudOut::Ptr NormalCloudOutPtr;
    typedef typename pcl::search::KdTree<PointT>::Ptr TreePtr;
    //typedef typename std::vector<PointCloudConstPtr>::const_iterator Iter;

  public:
    /**
     * \brief gets the field name to filter
     *
     * \return filter field name
     */
    const std::string& getFilterFieldName() const
    {
      return filter_field_name_;
    }
    /**
     * \brief sets the field name to filter
     *
     * \param filterFieldName
     */
    void setFilterFieldName(const std::string& filterFieldName)
    {
      filter_field_name_ = filterFieldName;
    }
    /**
     * \brief gets the filter limits on the filter field
     *
     * \return filter limits (max and min)
     */
    inline void getFilterLimits (float &limit_min, float &limit_max)
    {
      limit_max= filter_limit_max_;
      limit_min= filter_limit_min_;
    }
    /**
     * \brief sets the field name to filter
     *
     * \param limit_min filter limit minimum value
     * \param limit_max filter limit maximum value
     */
    inline void setFilterLimits (float &limit_min, float &limit_max)
    {
      filter_limit_max_ = limit_max;
      filter_limit_min_ = limit_min;
    }
    /**
     * \brief gets the vector of input clouds to concatenate
     *
     * \return input clouds
     */
    const std::vector<PointCloud>& getInputClouds() const
    {
      return input_clouds_;
    }
    /**
     * \brief sets the vector of input clouds to concatenate
     *
     * \param inputClouds vector of PointClods to concatenate
     */
    void setInputClouds(const std::vector<PointCloud>& inputClouds)
    {
      input_clouds_=inputClouds;
      /*for (iter_=inputClouds.begin(); iter_!=inputClouds.end(); ++iter_)
        {
          input_clouds_.push_back((*iter_));
          //ROS_INFO_STREAM("clouds has "<< (*iter_)->points.size() <<" points");
        }*/
    }

  public:
    /**
    * \brief Default constructor
    *
    * This method creates an filter
    *
    */
    ConcatenateMLS();
    /**
    * \brief Default destructor
    *
    */
    ~ConcatenateMLS();

  protected:
    /** \brief Filtered results are stored in point cloud.
    * \param[out] output The resultant point cloud.
    */
    void applyFilter (PointCloud &output);

    using pcl::Filter<PointT>::filter_name_;
    using pcl::PCLBase<PointT>::input_;

  private:
    //std::vector<PointCloudConstPtr> input_clouds_;
    /**
     * \brief vector of input PointCloud's
     */
    std::vector<PointCloud> input_clouds_;
    /**
     * \brief The desired user filter name (for dyn recon.)
     */
    std::string filter_field_name_;
    /**
     * \brief Minimum allowed value for filter (with user given name)
     */
    float filter_limit_min_;
    /**
     * \brief Maximum allowed value for filter (with user given name)
     */
    float filter_limit_max_;
    /**
     * \brief A cloud (constptr) to set as input to MLS
     */
    PointCloudConstPtr cloud_;
    /**
     * \brief The cloud (ptr) to hold the concatenated clouds
     */
    PointCloudPtr concat_cloud_;
    PointCloudPtr temp_concat_cloud_;
    /**
     * \brief The cloud (ptr) to hold the cloud to concatenate
     */
    PointCloud temp_cloud_;
    //PointCloudConstPtr temp_cloud_;
    //Iter iter_;
    /**
     * \brief The search tree for MLS
     */
    TreePtr tree_;
    /**
     * \brief Instance of pcl Moving Least Squares
     */
    pcl::MovingLeastSquares<PointT, pcl::PointNormal> mls_;
    /**
     * \brief The output normals from MLS averaging
     */
    NormalCloudOutPtr normals_;
    /**
     * \brief The search radius for MLS
     */
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
    /**
    * \brief Default constructor
    *
    * This method creates an filter
    *
    */
    ConcatenateMLS ();
    /**
    * \brief Default destructor
    *
    */
    ~ConcatenateMLS();
    /**
     * \brief gets the field name to filter
     *
     * \return filter field name
     */
    inline const std::string& getFilterFieldName() const
    {
      return filter_field_name_;
    }
    /**
     * \brief sets the field name to filter
     *
     * \param filterFieldName
     */
    inline void setFilterFieldName(const std::string& filterFieldName)
    {
      filter_field_name_ = filterFieldName;
    }
    /**
     * \brief gets the filter limits on the filter field
     *
     * \return filter limits (max and min)
     */
    inline void getFilterLimits (double &limit_min, double &limit_max)
    {
      limit_max= filter_limit_max_;
      limit_min= filter_limit_min_;
    }
    /**
     * \brief sets the field name to filter
     *
     * \param limit_min filter limit minimum value
     * \param limit_max filter limit maximum value
     */
    inline void setFilterLimits (const double &limit_min, const double &limit_max)
    {
      filter_limit_max_ = limit_max;
      filter_limit_min_ = limit_min;
    }
    /**
     * \brief gets the field name to filter
     *
     * \return filter field name
     */
    inline bool getKeepOrganized() const
    {
      return keep_organized_;
    }
    /** \brief Set whether the filtered points should be kept and set to the setUserFilterValue (default: NaN),
    * or removed from the PointCloud, thus potentially breaking its organized
    * structure. By default, points are removed.
    *
    * \param[in] keepOrganized set to true whether the filtered points should be kept and
    * set to a given user value (default: NaN)
    */
    inline void setKeepOrganized(bool keepOrganized)
    {
      keep_organized_ = keepOrganized;
    }
    /** \brief Provide a value that the filtered points should be set to
    * instead of removing them. Used in conjunction with setKeepOrganized ().
    * \param[in] val the user given value that the filtered point dimensions should be set to
    */
    inline void setUserFilterValue (float val)
    {
      user_filter_value_ = val;
    }
    /**
     * \brief gets the number of point clouds to concatenate
     *
     * \return integer number of pointclouds
     */
    int getNumImages() const
    {
      return num_images_;
    }
    /**
     * \brief sets the number of point clouds to concatenate
     *
     * \param numImages number of pointclouds
     */
    void setNumImages(int numImages)
    {
      num_images_ = numImages;
    }
    /**
     * \brief gets the name of the topic to listen to for PointCloud2 input
     *
     * \return string topic
     */
    const std::string& getTopic() const
    {
      return topic_;
    }
    /**
     * \brief sets the name of the topic to listen to for PointCloud2 input
     *
     * \param string topic
     */
    void setTopic(const std::string& topic)
    {
      topic_ = topic;
    }
    /**
     * \brief gets the search radius for moving least squares algorithm
     *
     * \return search radius
     */
    float getSearchRadius() const
    {
      return search_radius_;
    }
    /**
     * \brief sets the search radius for moving least squares algorithm
     *
     * \return float search radius
     */
    void setSearchRadius(float searchRadius)
    {
      search_radius_ = searchRadius;
    }

  protected:
    /** \brief Filtered results are stored in point cloud.
    * \param[out] output The resultant point cloud.
    */
    void applyFilter (sensor_msgs::PointCloud2 &output);

  private:
    /** \brief Keep the structure of the data organized, by setting the
    * filtered points to the a user given value (NaN by default).
    */
    bool keep_organized_;
    /**
     * \brief The desired user filter name (for dyn recon.)
     */
    std::string filter_field_name_;
    /**
     * \brief Minimum allowed value for filter (with user given name)
     */
    double filter_limit_min_;
    /**
     * \brief Minimum allowed value for filter (with user given name)
     */
    double filter_limit_max_;
    /**
     * \brief User input value to be set to any filtered point
     */
    float user_filter_value_;
    //std::vector<sensor_msgs::PointCloud2::ConstPtr> input_clouds_;
    /**
     * \brief Number of point clouds to concatenate
     */
    int num_images_;
    /**
     * \brief ROS topic for PointCloud2 data
     */
    std::string topic_;
    /**
     * \brief PointCloud2ConstPtr listening on topic to be converted and concatenated
     */
    sensor_msgs::PointCloud2::ConstPtr recent_cloud_;
    /**
     * \brief PointCloud converted from PointCloud2
     */
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    /**
     * \brief Vector of PointClouds which holds clouds to concatenate
     */
    //std::vector<pcl::PointCloud<pcl::PointXYZRGB> > clouds_;
    /**
     * \brief Minimum allowed value for filter (with user given name)
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr concat_cloud_;
    /**
     * \brief Minimum allowed value for filter (with user given name)

    pcl::PointCloud<pcl::PointXYZRGB> temp_cloud_;*/

    pcl::PointCloud<pcl::PointXYZRGB> output_pc_;
    /**
     * \brief A pointer to the search tree for MLS
     */
    TreePtr tree_;
    /**
     * \brief Implementation of MLS
     */
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls_;
    /**
     * \brief The point cloud that will hold the estimated normals from MLS averaging
     */
    NormalCloudOutPtr normals_;
    /**
     * \brief The search radius for MLS
     */
    float search_radius_;

  };
}

#endif /* CONCATENATE_MLS_H_ */
