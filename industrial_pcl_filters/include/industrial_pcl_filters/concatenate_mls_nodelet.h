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
 * concatenate_mls_nodelet.h
 *
 *  Created on: Jan 31, 2013
 *      Author: cgomez
 *
 *
 */

#ifndef CONCATENATE_MLS_NODELET_H_
#define CONCATENATE_MLS_NODELET_H_

#include "pcl_ros/filters/filter.h"
#include "concatenate_mls.h"
// Dynamic reconfigure
#include "industrial_pcl_filters/ConcatenateMLSConfig.h"

namespace industrial_pcl_filters_nodelets
{
  /** \brief @b ConcatenateMLS uses the base Filter class methods to pass through all data that satisfies the user given
    * constraints.
    * \author Christina Gomez
    */
  class ConcatenateMLS : public pcl_ros::Filter
  {
    protected:
      /** \brief Pointer to a dynamic reconfigure service. */
      boost::shared_ptr <dynamic_reconfigure::Server<industrial_pcl_filters::ConcatenateMLSConfig> > srv_;

      /** \brief Call the actual filter.
        * \param input the input point cloud
        * \param indices the input set of indices to use from \a input
        * \param output the resultant filtered dataset
        */
      inline void
      filter (const PointCloud2::ConstPtr &input, const IndicesPtr &indices,
              PointCloud2 &output)
      {
        boost::mutex::scoped_lock lock (mutex_);
        impl_.setInputCloud (input);
        impl_.setIndices (indices);
        impl_.filter (output);
      }

      /** \brief Child initialization routine.
        * \param nh ROS node handle
        * \param has_service set to true if the child has a Dynamic Reconfigure service
        */
      bool
      child_init (ros::NodeHandle &nh, bool &has_service);

      /** \brief Dynamic reconfigure service callback.
        * \param config the dynamic reconfigure FilterConfig object
        * \param level the dynamic reconfigure level
        */
      void
      config_callback (industrial_pcl_filters::ConcatenateMLSConfig &config, uint32_t level);

    private:
      /** \brief The PCL filter implementation used. */
      industrial_pcl_filters::ConcatenateMLS<PointCloud2> impl_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif /* CONCATENATE_MLS_NODELET_H_ */
