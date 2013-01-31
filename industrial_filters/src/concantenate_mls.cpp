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
 * concantenate_mls_nodelet.cpp
 *
 *
 * concantenate_mls.cpp
 *
 *  Created on: Jan 31, 2013
 *      Author: cgomez
 */

#include "concantenate_mls.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT>
industrial_filters::ConcantenateMLS<PointT>::ConcantenateMLS():
filter_limit_min_(FLT_MIN),
filter_limit_max_(FLT_MAX),
filter_field_name_("")
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
  //TODO actual filtering will go here
  output=*input_;

}
