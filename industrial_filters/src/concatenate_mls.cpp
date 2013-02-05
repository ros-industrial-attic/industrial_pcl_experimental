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
 * concantenate_mls.cpp
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

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
industrial_filters::ConcantenateMLS<PointT>::ConcantenateMLS():
filter_limit_min_(FLT_MIN),
filter_limit_max_(FLT_MAX),
filter_field_name_(""),
search_radius_(0.003)
{
  //Constructor
  filter_name_="ConcantenateMLS";
}
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
industrial_filters::ConcantenateMLS<PointT>::~ConcantenateMLS()
{
  //Destructor

}
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
void industrial_filters::ConcantenateMLS<PointT>::applyFilter(PointCloud &output)
{
  temp_cloud_=input_clouds_.at(0);
  int clouds_size = input_clouds_.size();
  for (int i=1; i<clouds_size; i++ )
  {
    //cloud += input_clouds_.at(i);
    pcl::concatenateFields(*temp_cloud_, *input_clouds_.at(i), *concat_cloud_);
    temp_cloud_= concat_cloud_;
  }
  cloud_=temp_cloud_;
  mls_.setOutputNormals(normals_);
  mls_.setInputCloud(cloud_);
  mls_.setPolynomialFit (true);
  mls_.setSearchMethod (tree_);
  mls_.setSearchRadius (search_radius_);

  mls_.reconstruct(output);
}

/*template <>
industrial_filters::ConcantenateMLS<pcl::PointXYZ>::ConcantenateMLS():
filter_limit_min_(FLT_MIN),
filter_limit_max_(FLT_MAX),
filter_field_name_(""),
search_radius_(0.003)
{
  //Constructor
  filter_name_="ConcantenateMLS";
}
*/
template class industrial_filters::ConcantenateMLS< pcl::PointXYZ >;
//template class industrial_filters::ConcantenateMLS< pcl::PointXYZRGB >;
//The macro PCL_INSTANTIATE does nothing else but go over a given list of types and creates an explicit instantiation for each
//#define PCL_INSTANTIATE_Concantenate(T) template class PCL_EXPORTS industrial_filters::ConcantenateMLS<T>;
//typedef industrial_filters::ConcantenateMLS ConcantenateMLS;
//#define PCL_INSTANTIATE(ConcantenateMLS, PCL_XYZ_POINT_TYPES);
//#define PCL_INSTANTIATE(ConcantenateMLS, POINT_TYPES)        \
//  BOOST_PP_SEQ_FOR_EACH(PCL_INSTANTIATE_IMPL, ConcantenateMLS, POINT_TYPES);
