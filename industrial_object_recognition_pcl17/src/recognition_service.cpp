/*
 * recognition_service.cpp
 *
 *  Created on: Apr 5, 2013
 *      Author: cgomez
 *
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
 */

#include <pcl17/io/pcd_io.h>
#include <pcl17/point_cloud.h>
#include <pcl17/correspondence.h>
#include <pcl17/features/normal_3d_omp.h>
#include <pcl17/features/shot_omp.h>
#include <pcl17/features/vfh.h>
#include <pcl17/features/board.h>
#include <pcl17/keypoints/uniform_sampling.h>
#include <pcl17/keypoints/sift_keypoint.h>

#include <pcl17/filters/passthrough.h>
#include <pcl17/filters/statistical_outlier_removal.h>
#include <pcl17/filters/filter.h>
//#include <industrial_pcl_filters/concatenate_mls.h>

#include <pcl17/recognition/cg/hough_3d.h>
#include <pcl17/recognition/cg/geometric_consistency.h>
#include <pcl17/registration/ia_ransac.h>
#include <pcl17/registration/icp.h>

#include <pcl17/kdtree/kdtree_flann.h>
#include <pcl17/kdtree/impl/kdtree_flann.hpp>
#include <pcl17/common/transforms.h>
#include <pcl17/console/parse.h>

#include <ros/ros.h>
#include <tabletop_object_detector/TabletopSegmentation.h>
#include <mantis_perception/mantis_recognition.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PoseArray.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <industrial_utils/param_utils.h>

#include <pcl17/apps/render_views_tesselated_sphere.h>
// VTK
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>

#include <iostream>

#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

typedef pcl17::PointXYZ PointType;
typedef pcl17::Normal NormalType;
typedef pcl17::ReferenceFrame RFType;
typedef pcl17::SHOT352 Descriptor1Type;
typedef pcl17::VFHSignature308 Descriptor2Type;

const std::string TABLETOP_SEGMENTATION = "ur5_arm/tabletop_segmentation";
std::ofstream STATFILE_("/home/jnicho/Desktop/pose_measurements.csv", ios::out | ios::app);
//STATFILE_.open("/home/jnicho/Desktop/pose_measurements.csv", ios::out | ios::app);

// Internal classes for organizing data
class CloudData
{
public:
  std::string name_;
  std::string source_name_;
  pcl17::PointCloud<PointType> points_;
  pcl17::PointCloud<NormalType> norms_;
  pcl17::PointCloud<PointType> keypoints_;
  pcl17::PointCloud<Descriptor1Type> descriptors1_;
  pcl17::PointCloud<Descriptor2Type> descriptors2_;
};

class RecognitionData
{
public:
  pcl17::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer_;
  pcl17::Hough3DGrouping<PointType, PointType, RFType, RFType> h_clusterer_;
  CloudData scene_;
  pcl17::Correspondences model_scene_corrs;
  std::vector<pcl17::Correspondence> cluster_corrs;
  float percent_match_;
};

class Recognizer
{
public:
  CloudData model_data_;
  RecognitionData rec_data_;
};

class RecognitionNode
{

public:

  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;
  //Publishers for intermediate steps
  ros::Publisher object_pub_, model_pub_, object_kp_pub_, model_kp_pub_, pose_pub_, grip_pose_pub_;
  std::string world_frame_;

  ros::ServiceClient tabletop_seg_client_;
  ros::ServiceServer recognition_server_;

  // Recognition
  std::vector<Recognizer> rec_list_;

  // Parameter values
  std::string model_path_;
  std::vector<std::string> model_list_;
  std::string ply_model_path_;
  std::string output_path_;
  //std::vector<std::string> model_list_;
  std::string model_name_;
  int best_match_;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses_;
  tf::Transform part_pose_;
  tf::Transform gripper_pose_;
  float part_rot_;
  int sample_number_;
  bool test_mode_;
  std::string pose_path_;

  pcl17::PointCloud<Descriptor1Type>::Ptr all_descriptors1_;
  pcl17::PointCloud<Descriptor2Type>::Ptr all_descriptors2_;
  std::vector<CloudData> models_;

  //const std::string stats_file_ = ply_model_path_ + "/mug.csv";
  //ROS_INFO_STREAM("Statistics file name: "<<stats_file.c_str());
  //ofstream statfile_ ("/home/cgomez/ros/fuerte/swri-ros-pkg_trunk_recognition_testing/mantis/mantis_perception/data/meshes/demo_parts");

  // Recognition parameters
  double model_ss_; //radius for model uniform sampling
  double scene_ss_; //radius for scene uniform sampling
  double descr_rad_; //radius for descriptor search
  double cg_size_; //correspondence grouping clustering size
  double cg_thresh_; //correspondence grouping clustering threshold
  double descr_nn_thresh_; //max distance between descriptors in nn search
  bool use_hough_; //true if using Hough for clustering correspondences, false if using geometric consistency
  double rf_rad_m_; //reference frame estimation radius for Hough - model
  double rf_rad_s_; //reference frame estimation radius for Hough - scene
  bool use_uniform_sampling_; //true if using Uniform Sampling for keypoint extraction, false if using SIFT keypoints
  double min_scale_; //parameter for SIFT
  int nr_octaves_; //parameter for SIFT
  int nr_scales_per_octave_; //parameter for SIFT
  double min_contrast_; //parameter for SIFT, mustbe non-negative
  bool use_VFH_; //true if using Viewpoint Feature Histogram as descriptor (vs SHOT)
  bool use_SACICP_; //true is using RANSAC for initial alignment
  double min_sample_distance_; //for RANSAC initial alignment
  int nr_iterations_; //for RANSAC initial alignment
  double max_corr_distance_; //for RANSAC initial alignment
  double transformation_epsilon_; //for ICP, leaving at 0.0
  int max_iterations_; //for ICP
  int tesselation_level_; //for render tesselated sphere to generate training data viewpoints
  double tess_view_angle_; //for render tesselated sphere to generate training data viewpoints - camera angle

  //Pre-processing parameters
  std::string topic_; //topic for input cloud
  double x_filter_min_; //for filtering input cloud, passthrough filter
  double x_filter_max_; //for filtering input cloud, passthrough filter
  double y_filter_min_; //for filtering input cloud, passthrough filter
  double y_filter_max_; //for filtering input cloud, passthrough filter
  double z_filter_min_; //for filtering input cloud, passthrough filter
  double z_filter_max_; //for filtering input cloud, passthrough filter
  double x_ee_offset_;
  double y_ee_offset_;
  double z_ee_offset_;
  int num_images_; //number of images to concatenate
  int sor_mean_; //statistical outlier removal
  double sor_thresh_; //statistical outlier removal

  RecognitionNode(ros::NodeHandle nh) :
      nh_(nh), priv_nh_("~"), model_ss_(0.01), scene_ss_(0.03), descr_rad_(0.02), cg_size_(0.01), cg_thresh_(5.0), descr_nn_thresh_(
          0.25), use_hough_(false), rf_rad_m_(0.015), rf_rad_s_(0.015),use_uniform_sampling_(true), min_scale_(0.0005), nr_octaves_(4), nr_scales_per_octave_(
          5), min_contrast_(1), use_VFH_(true), use_SACICP_(true), min_sample_distance_(0.0), nr_iterations_(3), transformation_epsilon_(
          0.0), max_iterations_(1000), max_corr_distance_(0.05), x_filter_min_(-1.0), x_filter_max_(1.0), y_filter_min_(
          -1.0), y_filter_max_(1.0), z_filter_min_(-1.0), z_filter_max_(1.0), tesselation_level_(1), tess_view_angle_(
          60.0), sor_mean_(10), sor_thresh_(1.0), num_images_(1), sample_number_(0), test_mode_(false)
  {
    //tabletop_seg_client_ = nh_.serviceClient<tabletop_object_detector::TabletopSegmentation>(TABLETOP_SEGMENTATION);

    object_pub_ = nh.advertise<sensor_msgs::PointCloud2>("object_cluster", 1);
    model_pub_ = nh.advertise<sensor_msgs::PointCloud2>("model_cluster", 1);
    object_kp_pub_ = nh.advertise<sensor_msgs::PointCloud2>("object_kp", 1);
    model_kp_pub_ = nh.advertise<sensor_msgs::PointCloud2>("model_kp", 1);
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("model_pose", 1);
    grip_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("grasp_pose", 1);

  }

  ~RecognitionNode()
  {
  }

  bool init()
  {
    if (!priv_nh_.getParam("ply_model_path", ply_model_path_))
    {
      ROS_ERROR("Failed to get model_path parameter, can't find training data");
      return false;
    }
    else
    {
      ROS_INFO_STREAM("Loaded model path: " << ply_model_path_);
    }

    if (!industrial_utils::param::getListParam("~model_list", model_list_))
    {
      ROS_ERROR("Failed to get model_list parameter, can't load training models");
      return false;
    }
    else
    {
      ROS_INFO_STREAM("Loaded filenames for " << model_list_.size() << " training models");
    }

    priv_nh_.getParam("world_frame", world_frame_);
    priv_nh_.getParam("model_name", model_name_);
    priv_nh_.getParam("model_path", model_path_);
    priv_nh_.getParam("model_ss", model_ss_);
    priv_nh_.getParam("scene_ss", scene_ss_);
    //priv_nh_.getParam("descr_rad", descr_rad_);
    priv_nh_.getParamCached("descr_rad", descr_rad_);
    //priv_nh_.getParam("cg_size", cg_size_);
    priv_nh_.getParamCached("cg_size", cg_size_);
    priv_nh_.getParam("cg_thresh", cg_thresh_);
    //priv_nh_.getParam("descr_nn_thresh", descr_nn_thresh_);
    priv_nh_.getParamCached("descr_nn_thresh", descr_nn_thresh_);
    priv_nh_.getParam("use_hough", use_hough_);
    //priv_nh_.getParam("rf_radius", rf_rad_);
    priv_nh_.getParamCached("rf_radius_m", rf_rad_m_);
    priv_nh_.getParamCached("rf_radius_s", rf_rad_s_);
    priv_nh_.getParam("use_SACICP", use_SACICP_);
    priv_nh_.getParam("use_uniform_sampling", use_uniform_sampling_);
    priv_nh_.getParam("use_VFH", use_VFH_);
    priv_nh_.getParam("min_scale", min_scale_);
    priv_nh_.getParam("nr_octaves", nr_octaves_);
    priv_nh_.getParam("nr_scales_per_octave", nr_scales_per_octave_);
    priv_nh_.getParam("min_contrast", min_contrast_);
    priv_nh_.getParam("min_sample_distance", min_sample_distance_);
    priv_nh_.getParam("max_corr_distance", max_corr_distance_);
    priv_nh_.getParam("nr_iterations", nr_iterations_);
    priv_nh_.getParam("transformation_epsilon", transformation_epsilon_);
    priv_nh_.getParam("max_iteration", max_iterations_);
    priv_nh_.getParam("x_filter_min", x_filter_min_);
    priv_nh_.getParam("x_filter_max", x_filter_max_);
    priv_nh_.getParam("y_filter_min", y_filter_min_);
    priv_nh_.getParam("y_filter_max", y_filter_max_);
    priv_nh_.getParam("z_filter_min", z_filter_min_);
    priv_nh_.getParam("z_filter_max", z_filter_max_);
    priv_nh_.getParam("x_ee_offset", x_ee_offset_);
    priv_nh_.getParam("y_ee_offset", y_ee_offset_);
    priv_nh_.getParam("z_ee_offset", z_ee_offset_);
    priv_nh_.getParam("tesselation_level", tesselation_level_);
    priv_nh_.getParam("tess_view_angle", tess_view_angle_);
    priv_nh_.getParam("topic", topic_);
    priv_nh_.getParam("sor_mean", sor_mean_);
    priv_nh_.getParam("sor_thresh", sor_thresh_);
    priv_nh_.getParam("num_images", num_images_);
    priv_nh_.getParam("last_sample_number", sample_number_);
    priv_nh_.getParam("pose_path", pose_path_);
    priv_nh_.getParam("test_mode", test_mode_);

    ROS_INFO("--- Recognition Parameters ---");
    ROS_INFO_STREAM("model_ss: " << model_ss_);
    ROS_INFO_STREAM("scene_ss: " << scene_ss_);
    ROS_INFO_STREAM("descr_rad: " << descr_rad_);
    ROS_INFO_STREAM("cg_size: " << cg_size_);
    ROS_INFO_STREAM("cg_thresh: " << cg_thresh_);
    ROS_INFO_STREAM("descr_nn_thresh: " << descr_nn_thresh_);
    ROS_INFO_STREAM("Using Hough for clustering: " << use_hough_);
    ROS_INFO_STREAM("Using Uniform Sampling for keypoints: " << use_uniform_sampling_);
    ROS_INFO_STREAM("RF radius for model: " << rf_rad_m_);
    ROS_INFO_STREAM("RF radius for scene: " << rf_rad_s_);


    if (STATFILE_.is_open())
    {
      STATFILE_ << "Roll, Pitch, Yaw, "
          "x, y, z  \n";
    }
    else
    {
      ROS_INFO_STREAM("Unable to open file");
      return false;
    }

    if (!initRecognizers())
    {
      return false;
    }
    /*
     if (!tabletop_seg_client_.waitForExistence())
     {
     ROS_ERROR("Failed to find table top segmentation service");
     return false;
     }
     else
     {
     ROS_INFO("Connected to service");
     }
     */
    return true;

  }

  bool initRecognizers()
  {
    if (model_list_.size() > 0)
    {
      rec_list_.clear();
      ros::Time start_init = ros::Time::now();
      for (std::size_t i = 0; i < model_list_.size(); i++)
      {
        std::string model_name = model_list_[i];
        ROS_INFO_STREAM("--- Loading " << model_name << " ---");

        Recognizer rec;

        if (!initModelData(model_name, rec.model_data_))
        {
          ROS_ERROR_STREAM("Failed to load model: " << model_name);
          return false;
        }
        ROS_INFO("Initializing recognizer");
        pcl17::PointCloud<PointType>::Ptr model_ptr(new pcl17::PointCloud<PointType>(rec.model_data_.points_));
        if (use_hough_)
        {
          rec.rec_data_.h_clusterer_.setHoughBinSize(cg_size_);
          rec.rec_data_.h_clusterer_.setHoughThreshold(cg_thresh_);
          ROS_INFO_STREAM(
              "Recognizer config - GC Size: " << rec.rec_data_.h_clusterer_.getHoughBinSize()<< " GC Threshold: " << rec.rec_data_.h_clusterer_.getHoughThreshold());
        }
        else
        {
          rec.rec_data_.gc_clusterer_.setGCSize(cg_size_);
          rec.rec_data_.gc_clusterer_.setGCThreshold(cg_thresh_);
          ROS_INFO_STREAM(
              "Recognizer config - GC Size: " << rec.rec_data_.gc_clusterer_.getGCSize() << " GC Threshold: " << rec.rec_data_.gc_clusterer_.getGCThreshold());
        }
        rec_list_.push_back(rec);
      }
      ROS_INFO_STREAM(rec_list_.size() << " recognizers successfully initialized");
      ros::Time finish_init = ros::Time::now();
      ros::Duration total_init = finish_init - start_init;
      ROS_INFO_STREAM("Initializing took "<< total_init<<" s");
    }
    else
    {
      ROS_ERROR_STREAM("No files to load for recognition training");
      return false;
    }

    return true;
  }

  bool initModelData(std::string & name, CloudData & model_data)
  {
    ROS_INFO_STREAM("Initialize model data");

    std::vector<std::string> filenames;
    all_descriptors1_ = pcl17::PointCloud<Descriptor1Type>::Ptr(new pcl17::PointCloud<Descriptor1Type>);
    all_descriptors2_ = pcl17::PointCloud<Descriptor2Type>::Ptr(new pcl17::PointCloud<Descriptor2Type>);

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    pcl17::apps::RenderViewsTesselatedSphere render_views;
    //std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > poses;
    // represents the pose of the cloud relative to the model coordinate system
    std::vector<pcl17::PointCloud<PointType>::Ptr> views;

    std::string filename = ply_model_path_ + "/" + model_list_.at(0);

    /*reader->SetFileName(filename.c_str());
     mapper->SetInputConnection(reader->GetOutputPort());
     mapper->Update();
     vtkSmartPointer<vtkPolyData> pdata = vtkSmartPointer<vtkPolyData>::New();
     pdata->Allocate(mapper->GetInput());
     render_views.addModelFromPolyData(mapper->GetInput());
     render_views.setTesselationLevel(tesselation_level_);
     render_views.setViewAngle(tess_view_angle_);
     render_views.generateViews();
     render_views.getViews(views);
     render_views.getPoses(poses_);*/

    //IF READING .pcd's FROM DIRECTORY
    struct dirent *dp;
    DIR *dfd;

    const char *dir;
    dir = model_path_.c_str();
    ROS_INFO_STREAM("Reading files from directory: "<<dir);

    if ((dfd = opendir(dir)) == NULL)
    {
      ROS_ERROR_STREAM("Can't open "<< dir);
    }
    char filename_qfd[200];
    while ((dp = readdir(dfd)) != NULL)
    {
      struct stat stbuf;
      sprintf(filename_qfd, "%s/%s", dir, dp->d_name);
      if (stat(filename_qfd, &stbuf) == -1)
      {
        printf("Unable to stat file: %s\n", filename_qfd);
        continue;
      }

      if ((stbuf.st_mode & S_IFMT) == S_IFDIR)
      {
        continue;
        // Skip directories
      }
      else
      {
        //Do real stuff here, like make array of filenames
        char file_name[100];
        sprintf(file_name, dp->d_name);
        //ROS_INFO_STREAM("File being stored to filenames array: "<< file_name);
        filenames.push_back(file_name);
      }
    }

    ROS_INFO_STREAM("List of .pcd files created with "<<filenames.size()<< " files");

    // Parse the exemplar files
    pcl17::PointCloud<PointType> cloud = pcl17::PointCloud<PointType>();
    //int n = views.size();
    int n = filenames.size();
    ROS_INFO_STREAM("Tesselated sphere generated "<<n<<" sets of training clouds and matrices");
    models_.resize(n);
    for (int i = 0; i < n; i++)
    {

      std::string filename = model_path_ + "/" + filenames[i];

      ROS_INFO_STREAM("Loading cloud data from " << filename);
      if (pcl17::io::loadPCDFile(filename, cloud) < 0)
      {
        ROS_ERROR_STREAM("Error loading model: " << filename);
        return false;
      }

      int lastindex = filename.find_last_of(".");
      name = filename.substr(0, lastindex);

      std::string name;
      std::stringstream num;
      num << i;
      name = model_name_ + num.str();
      //cloud = *views[i];

      ROS_INFO_STREAM( "Initializing model: " << name << "with " << cloud.size() << " points");

      initCloudData(model_ss_, name, filename, cloud, models_[i]);
      num.str("");
      if (use_VFH_)
      {
        *all_descriptors2_ += models_[i].descriptors2_;
      }
      else
      {
        *all_descriptors1_ += models_[i].descriptors1_;
      }
    }
    //return initCloudData(model_ss_, name, filename, cloud, model_data);
    ROS_INFO_STREAM("Initialized "<<all_descriptors2_->size()<<" VFH models");
    ROS_INFO_STREAM("Initialized "<<all_descriptors1_->size()<<" SHOT models");
    return true;
  }

  bool initCloudData(float radius_search, std::string & name, std::string & source_name,
                     pcl17::PointCloud<PointType> & cloud, CloudData & data)
  {
	ROS_INFO_STREAM("Inside initCloudData");
    // Clear out an existing data
    data = CloudData();

    data.name_ = name;
    ROS_INFO_STREAM("Initializing model data for " << data.name_);

    data.source_name_ = source_name;
    ROS_INFO_STREAM("Loading data from source: " << data.source_name_);

    data.points_ = cloud;
    ROS_INFO_STREAM( "Copying cloud: " << name << "with " << data.points_.size() << " points");

    // Making a local copy for the setInputCloud methods below
    pcl17::PointCloud<PointType>::Ptr model_ptr(new pcl17::PointCloud<PointType>(data.points_));
    ;
    ROS_INFO("Estimating model normals");
    pcl17::NormalEstimationOMP<PointType, NormalType> norm_est;
    norm_est.setKSearch(10);
    norm_est.setInputCloud(model_ptr);
    norm_est.compute(data.norms_);

    ROS_INFO_STREAM( "Calculated normals for: " << data.name_ << " with " << data.norms_.size() << " normals");

    pcl17::PointCloud<int> sampled_indices;
    pcl17::PointCloud<NormalType>::Ptr norms_ptr(new pcl17::PointCloud<NormalType>(data.norms_));

    if (use_uniform_sampling_)
    {
      ROS_INFO("Uniform sampling for keypoints");
      pcl17::UniformSampling<PointType> uniform_sampling;
      uniform_sampling.setInputCloud(model_ptr);
      uniform_sampling.setRadiusSearch(radius_search);
      uniform_sampling.compute(sampled_indices);
      pcl17::copyPointCloud(*model_ptr, sampled_indices.points, data.keypoints_);
      ROS_INFO_STREAM(
          "Uniform sampling: " << data.name_ << " resulting in " << data.keypoints_.size() << " US keypoints");
    }
    else
    {
      ROS_INFO("SIFT Keypoints");
      pcl17::SIFTKeypoint<pcl17::PointNormal, pcl17::PointWithScale> sift;
      pcl17::search::KdTree<pcl17::PointNormal>::Ptr SIFT_tree(new pcl17::search::KdTree<pcl17::PointNormal>());
      ;		//new API
      pcl17::PointCloud<pcl17::PointWithScale>::Ptr sifts(new pcl17::PointCloud<pcl17::PointWithScale>);
      pcl17::PointCloud<pcl17::PointNormal>::Ptr cloud_with_normals(new pcl17::PointCloud<pcl17::PointNormal>);
      pcl17::concatenateFields(*model_ptr, *norms_ptr, *cloud_with_normals);
      sift.setInputCloud(cloud_with_normals);
      sift.setSearchMethod(SIFT_tree);
      sift.setScales(min_scale_, nr_octaves_, nr_scales_per_octave_);
      sift.setMinimumContrast(min_contrast_);
      sift.compute(*sifts);
      pcl17::copyPointCloud(*sifts, data.keypoints_);

      ROS_INFO_STREAM("Computed " << sifts->points.size () << " SIFT Keypoints");
    }
    if (use_VFH_)
    {
      pcl17::VFHEstimation<PointType, NormalType, Descriptor2Type> vfh;
      vfh.setInputCloud(model_ptr);
      vfh.setInputNormals(norms_ptr);
      pcl17::search::KdTree<PointType>::Ptr VFH_tree(new pcl17::search::KdTree<PointType>());
      vfh.setSearchMethod(VFH_tree);
      vfh.compute(data.descriptors2_);
      ROS_INFO_STREAM(
          "VFH descriptors calculated: " << data.name_ << " resulting in " << data.descriptors2_.size() << " descriptors");
    }
    else
    {
      ROS_INFO("SHOT descriptors");
      pcl17::PointCloud<PointType>::Ptr keypoints_ptr(new pcl17::PointCloud<PointType>(data.keypoints_));

      pcl17::SHOTEstimationOMP<PointType, NormalType, Descriptor1Type> descr_est;
      priv_nh_.getParamCached("descr_rad", descr_rad_);
      descr_est.setRadiusSearch(descr_rad_);

      descr_est.setInputCloud(keypoints_ptr);
      descr_est.setInputNormals(norms_ptr);
      descr_est.setSearchSurface(model_ptr);
      descr_est.compute(data.descriptors1_);
      ROS_INFO_STREAM(
          " SHOT descriptors calculated: " << data.name_ << " resulting in " << data.descriptors1_.size() << " descriptors");
    }

    return true;
  }

  bool recognize(pcl17::PointCloud<PointType> & scene)
  {
    use_VFH_ = true;
    //for (size_t i = 0; i < rec_list_.size(); i++)
    //{
    return recognizeModel(scene, rec_list_[0]);
    //}

  }

  bool recognizeModel(pcl17::PointCloud<PointType> & scene, Recognizer & rec)
  {
    ROS_INFO_STREAM("#####################################################");
    bool success(false);
    std::string name("scene");
    std::string source_name("streaming");

    rec.rec_data_.cluster_corrs.clear();
    rec.rec_data_.model_scene_corrs.clear();
    rec.rec_data_.percent_match_ = 0.0;
    ROS_INFO_STREAM("Things initialized for scene");
    ros::Time init_scene_start = ros::Time::now();
    initCloudData(scene_ss_, name, source_name, scene, rec.rec_data_.scene_);
    ROS_INFO_STREAM("scene data initialized");
    ros::Time init_scene_finish = ros::Time::now();
    ros::Duration init_scene_total = init_scene_finish - init_scene_start;
    ROS_INFO_STREAM("First pass over scene for VFH descriptors took "<< init_scene_total<<" s");
    ros::Time vfh_start = ros::Time::now();
    if (use_VFH_)
    {
      pcl17::KdTreeFLANN<Descriptor2Type>::Ptr VFH_search(new pcl17::KdTreeFLANN<Descriptor2Type>);
      pcl17::KdTreeFLANN<Descriptor2Type> vfh_match_search;
      //pcl17::PointCloud<DescriptorType>::Ptr descriptors_ptr(new pcl17::PointCloud<DescriptorType>(all_descriptors_));
      VFH_search->setInputCloud(all_descriptors2_);

      ROS_INFO_STREAM("Search Tree set up with "<< all_descriptors2_->size()<<" descriptors from model data");

      std::vector<int> nn_index(1);
      std::vector<float> nn_sqr_dists(1);
      const Descriptor2Type query_descriptor = rec.rec_data_.scene_.descriptors2_.points[0]; // .global_descriptor->points[0];
      VFH_search->nearestKSearch(query_descriptor, 1, nn_index, nn_sqr_dists);
      best_match_ = nn_index[0];
      ROS_INFO_STREAM("Found best match at "<<best_match_<<" about to set model_data to models["<<best_match_<<"]...");
      //rec.model_data_=models_[best_match];

      std::string file = models_[best_match_].source_name_;
      std::string nm = models_[best_match_].name_;
      pcl17::PointCloud<PointType> mod_cloud = models_[best_match_].points_;

      ros::Time vfh_finish = ros::Time::now();
      ros::Duration vfh_total = vfh_finish - vfh_start;
      ROS_INFO_STREAM("VFH description took "<< vfh_total<<" s");

      ros::Time init_both_start = ros::Time::now();
      use_VFH_ = false;
      initCloudData(model_ss_, nm, file, mod_cloud, rec.model_data_);
      ros::Time init_model_finish = ros::Time::now();
      ros::Duration init_model_total = init_model_finish - init_both_start;
      initCloudData(scene_ss_, name, source_name, scene, rec.rec_data_.scene_);
      ros::Time init_both_finish = ros::Time::now();
      ros::Duration init_both_total = init_both_finish - init_both_start;
      ros::Duration init_sc_total = init_both_finish - init_model_finish;
      ROS_INFO_STREAM("Second pass over both model and scene for SHOT descriptors took "<< init_both_total<<" s");
      ROS_INFO_STREAM("SHOT descriptors model took "<< init_model_total<<" s for "<<mod_cloud.size()<<" points");
      ROS_INFO_STREAM("SHOT descriptors model took "<< init_sc_total<<" s for "<<scene.size()<<" points");
    }

    sensor_msgs::PointCloud2 object_kp_msg;
    pcl17::toROSMsg(rec.rec_data_.scene_.keypoints_, object_kp_msg);
    object_kp_msg.header.frame_id = world_frame_;
    object_kp_pub_.publish(object_kp_msg);
    //At this point in the code you have a scene cloud matched to a model cloud (one from source directory)
    //which was matched via VFH histogram kdtree search, and subsequently, you have the SHOT descriptors for
    //the scene cloud and said model cloud. Now choose from registration via RANCAS_IA combined with ICP,
    //or Correspondence matching via Hough or GC

    if (use_SACICP_)
    {

      pcl17::PointCloud<PointType>::Ptr model_pt_ptr(new pcl17::PointCloud<PointType>(rec.model_data_.points_));
      pcl17::PointCloud<PointType>::Ptr scene_pt_ptr(new pcl17::PointCloud<PointType>(rec.rec_data_.scene_.points_));
      pcl17::PointCloud<PointType>::Ptr model_kp_ptr(new pcl17::PointCloud<PointType>(rec.model_data_.keypoints_));
      pcl17::PointCloud<PointType>::Ptr scene_kp_ptr(new pcl17::PointCloud<PointType>(rec.rec_data_.scene_.keypoints_));
      pcl17::PointCloud<Descriptor1Type>::Ptr model_desc_ptr(
          new pcl17::PointCloud<Descriptor1Type>(rec.model_data_.descriptors1_));
      pcl17::PointCloud<Descriptor1Type>::Ptr scene_desc_ptr(
          new pcl17::PointCloud<Descriptor1Type>(rec.rec_data_.scene_.descriptors1_));
      pcl17::SampleConsensusInitialAlignment<PointType, PointType, Descriptor1Type> sac_ia;
      pcl17::PointCloud<PointType> registration_output;

      sac_ia.setMinSampleDistance(min_sample_distance_);
      sac_ia.setMaximumIterations(nr_iterations_);
      sac_ia.setMaxCorrespondenceDistance(max_corr_distance_);

      //sac_ia.setInputCloud(model_kp_ptr);
      sac_ia.setInputSource(model_kp_ptr);
      sac_ia.setSourceFeatures(model_desc_ptr);
      sac_ia.setInputTarget(scene_kp_ptr);
      sac_ia.setTargetFeatures(scene_desc_ptr);

      sac_ia.align(registration_output);

      Eigen::Matrix4f sac_ia_tform;
      sac_ia_tform = sac_ia.getFinalTransformation();

      pcl17::IterativeClosestPoint<PointType, PointType> icp;
      pcl17::PointCloud<PointType>::Ptr source_points_transformed(new pcl17::PointCloud<PointType>);
      pcl17::PointCloud<PointType> icp_registration_output;
      //icp.setMaxCorrespondenceDistance()
      //icp.setRANSACOutlierRejectionThreshold()
      icp.setTransformationEpsilon(transformation_epsilon_);
      icp.setMaximumIterations(max_iterations_);

      pcl17::transformPointCloud(*model_pt_ptr, *source_points_transformed, sac_ia_tform);

      //icp.setInputCloud(source_points_transformed);
      icp.setInputSource(source_points_transformed);
      icp.setInputTarget(scene_pt_ptr);
      icp.align(icp_registration_output);
      Eigen::Matrix4f icp_tform;
      icp_tform = icp.getFinalTransformation() * sac_ia_tform;

      ROS_INFO_STREAM("Registration complete");
      ROS_WARN_STREAM("Alignment score: "<<icp.getFitnessScore());

      pcl17::PointCloud<PointType> transformed_model;
      pcl17::transformPointCloud(rec.model_data_.points_, transformed_model, icp_tform);

      sensor_msgs::PointCloud2 model_msg;
      pcl17::toROSMsg(transformed_model, model_msg);
      model_msg.header.frame_id = world_frame_;
      //ROS_INFO_STREAM("Publishing " << model_msg.width << " points in "
      //                << model_msg.header.frame_id << " frame");
      model_pub_.publish(model_msg);

      pcl17::PointCloud<PointType> transformed_model_kp;
      pcl17::transformPointCloud(rec.model_data_.keypoints_, transformed_model_kp, icp_tform);
      ROS_INFO_STREAM("Model keypoints cloud size "<<rec.model_data_.keypoints_.size());
      sensor_msgs::PointCloud2 model_kp_msg;
      pcl17::toROSMsg(transformed_model_kp, model_kp_msg);
      model_kp_msg.header.frame_id = world_frame_;
      model_kp_pub_.publish(model_kp_msg);

      ros::Timer myTime;
      ros::Duration(2.0).sleep();
    }
    else
    {
      ros::Time registration_start = ros::Time::now();
      //pcl17::KdTreeFLANN<Descriptor1Type>::Ptr SHOT_search (new pcl17::KdTreeFLANN<Descriptor1Type>);
      pcl17::KdTreeFLANN<Descriptor1Type> shot_match_search;
      //pcl17::PointCloud<Descriptor1Type>::Ptr model_descriptors_ptr(new pcl17::PointCloud<Descriptor1Type>(rec.model_data_.descriptors1_));
      //SHOT_search->setInputCloud(model_descriptors_ptr);

      pcl17::PointCloud<Descriptor1Type>::Ptr model_shot_descriptors_ptr(
          new pcl17::PointCloud<Descriptor1Type>(rec.model_data_.descriptors1_));
      shot_match_search.setInputCloud(model_shot_descriptors_ptr);
      priv_nh_.getParamCached("descr_nn_thresh", descr_nn_thresh_);
      //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
      for (size_t i = 0; i < rec.rec_data_.scene_.descriptors1_.size(); ++i)
      {
        std::vector<int> neigh_indices(1);
        std::vector<float> neigh_sqr_dists(1);
        if (!pcl_isfinite (rec.rec_data_.scene_.descriptors1_.at(i).descriptor[0])) //skipping NaNs
        {
          continue;
        }
        int found_neighs = shot_match_search.nearestKSearch(rec.rec_data_.scene_.descriptors1_.at(i), 1, neigh_indices,
                                                            neigh_sqr_dists);
        if (found_neighs == 1 && neigh_sqr_dists[0] < descr_nn_thresh_) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
          pcl17::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
          rec.rec_data_.model_scene_corrs.push_back(corr);
        }
      }
      ros::Time correspondence_finish = ros::Time::now();
      ros::Duration correspondence_total = correspondence_finish - registration_start;
      ROS_INFO_STREAM("Finding Model/Scene correspondences took "<< correspondence_total<<" s");

      ROS_INFO_STREAM("Model/Scene correspondences found: " << rec.rec_data_.model_scene_corrs.size());

      pcl17::PointCloud<PointType>::Ptr model_kp_ptr(new pcl17::PointCloud<PointType>(rec.model_data_.keypoints_));
      //pcl17::PointCloud<PointType>::Ptr model_kp_ptr(new pcl17::PointCloud<PointType>(all_keypoints_));
      pcl17::PointCloud<PointType>::Ptr scene_kp_ptr(new pcl17::PointCloud<PointType>(rec.rec_data_.scene_.keypoints_));
      pcl17::CorrespondencesPtr model_scene_corrs(new pcl17::Correspondences(rec.rec_data_.model_scene_corrs));

      pcl17::PointCloud<NormalType>::Ptr model_norm_ptr(new pcl17::PointCloud<NormalType>(rec.model_data_.norms_));
      //pcl17::PointCloud<NormalType>::Ptr model_norm_ptr(new pcl17::PointCloud<NormalType>(all_normals_));
      pcl17::PointCloud<PointType>::Ptr model_pt_ptr(new pcl17::PointCloud<PointType>(rec.model_data_.points_));
      pcl17::PointCloud<NormalType>::Ptr scene_norm_ptr(new pcl17::PointCloud<NormalType>(rec.rec_data_.scene_.norms_));
      pcl17::PointCloud<PointType>::Ptr scene_pt_ptr(new pcl17::PointCloud<PointType>(rec.rec_data_.scene_.points_));

      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
      std::vector<pcl17::Correspondences> clustered_corrs;

      if (use_hough_)
      {
        ros::Time hough_start = ros::Time::now();
        priv_nh_.getParamCached("rf_radius", rf_rad_m_);
        priv_nh_.getParamCached("rf_radius", rf_rad_s_);
        pcl17::PointCloud<RFType>::Ptr model_rf(new pcl17::PointCloud<RFType>());
        pcl17::PointCloud<RFType>::Ptr scene_rf(new pcl17::PointCloud<RFType>());
        pcl17::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
        rf_est.setFindHoles(true);
        rf_est.setRadiusSearch(rf_rad_m_);

        rf_est.setInputCloud(model_kp_ptr);
        rf_est.setInputNormals(model_norm_ptr);
        rf_est.setSearchSurface(model_pt_ptr);
        rf_est.compute(*model_rf);

        rf_est.setRadiusSearch(rf_rad_s_);
        rf_est.setInputCloud(scene_kp_ptr);
        rf_est.setInputNormals(scene_norm_ptr);
        rf_est.setSearchSurface(scene_pt_ptr);
        rf_est.compute(*scene_rf);

        priv_nh_.getParamCached("cg_size", cg_size_);
        rec.rec_data_.h_clusterer_.setHoughBinSize(cg_size_);
        rec.rec_data_.h_clusterer_.setHoughThreshold(cg_thresh_);
        rec.rec_data_.h_clusterer_.setUseInterpolation(true);
        rec.rec_data_.h_clusterer_.setUseDistanceWeight(false);

        rec.rec_data_.h_clusterer_.setInputCloud(model_kp_ptr);
        rec.rec_data_.h_clusterer_.setInputRf(model_rf);
        rec.rec_data_.h_clusterer_.setSceneCloud(scene_kp_ptr);
        rec.rec_data_.h_clusterer_.setSceneRf(scene_rf);
        rec.rec_data_.h_clusterer_.setModelSceneCorrespondences(model_scene_corrs);

        rec.rec_data_.h_clusterer_.recognize(rototranslations, clustered_corrs);

        ros::Time hough_finish = ros::Time::now();
        ros::Duration hough_total = hough_finish - hough_start;
        ROS_INFO_STREAM("Finding hough cluster took "<< hough_total<<" s");

        if (cg_thresh_ == -1)
        {
          rec.rec_data_.percent_match_ = float(clustered_corrs[0].size())
              / float(rec.rec_data_.model_scene_corrs.size());
          ros::Time icp_start = ros::Time::now();
          pcl17::PointCloud<PointType>::Ptr model_pt_ptr(new pcl17::PointCloud<PointType>(rec.model_data_.points_));
          pcl17::PointCloud<PointType>::Ptr scene_pt_ptr(
              new pcl17::PointCloud<PointType>(rec.rec_data_.scene_.points_));
          pcl17::PointCloud<PointType>::Ptr model_kp_ptr(new pcl17::PointCloud<PointType>(rec.model_data_.keypoints_));
          pcl17::PointCloud<PointType>::Ptr scene_kp_ptr(
              new pcl17::PointCloud<PointType>(rec.rec_data_.scene_.keypoints_));
          pcl17::PointCloud<Descriptor1Type>::Ptr model_desc_ptr(
              new pcl17::PointCloud<Descriptor1Type>(rec.model_data_.descriptors1_));
          pcl17::PointCloud<Descriptor1Type>::Ptr scene_desc_ptr(
              new pcl17::PointCloud<Descriptor1Type>(rec.rec_data_.scene_.descriptors1_));
          pcl17::SampleConsensusInitialAlignment<PointType, PointType, Descriptor1Type> sac_ia;
          pcl17::PointCloud<PointType> registration_output;

          pcl17::IterativeClosestPoint<PointType, PointType> icp;
          pcl17::PointCloud<PointType>::Ptr source_points_transformed(new pcl17::PointCloud<PointType>);
          pcl17::PointCloud<PointType> icp_registration_output;
          //icp.setMaxCorrespondenceDistance()
          //icp.setRANSACOutlierRejectionThreshold()
          icp.setTransformationEpsilon(transformation_epsilon_);
          icp.setMaximumIterations(max_iterations_);

          Eigen::Matrix4f hough_tform;
          hough_tform = rototranslations[0];
          pcl17::transformPointCloud(*model_pt_ptr, *source_points_transformed, hough_tform);

          //icp.setInputCloud(source_points_transformed);
          icp.setInputSource(source_points_transformed);
          icp.setInputTarget(scene_pt_ptr);
          icp.align(icp_registration_output);
          Eigen::Matrix4f icp_tform;
          icp_tform = icp.getFinalTransformation() * hough_tform;
          rototranslations[0] = icp_tform;

          ROS_INFO_STREAM("Registration complete");
          ROS_WARN_STREAM("Hough followed by ICP Alignment score: "<<icp.getFitnessScore());
          ROS_WARN_STREAM("Hough followed by ICP max distance: "<<icp.getMaxCorrespondenceDistance());
          ros::Time icp_finish = ros::Time::now();
          ros::Duration icp_total = icp_finish - icp_start;
          ROS_INFO_STREAM("ICP took "<< icp_total<<" s");
        }
        ros::Time registration_finish = ros::Time::now();
        ros::Duration registration_total = registration_finish - registration_start;
        ROS_INFO_STREAM("Finding and matching Model/Scene correspondences took "<< registration_total<<" s");
      }
      else
      {

        rec.rec_data_.gc_clusterer_.setInputCloud(model_kp_ptr);
        rec.rec_data_.gc_clusterer_.setSceneCloud(scene_kp_ptr);
        rec.rec_data_.gc_clusterer_.setModelSceneCorrespondences(model_scene_corrs);
        rec.rec_data_.gc_clusterer_.setGCSize(cg_size_);
        rec.rec_data_.gc_clusterer_.setGCThreshold(cg_thresh_);

        rec.rec_data_.gc_clusterer_.recognize(rototranslations, clustered_corrs);
      }

      ROS_INFO_STREAM("========================================================");
      ROS_INFO_STREAM("Recognizer: " << rec.model_data_.name_ << " found " << rototranslations.size() << " instances");

      for (size_t j = 0; j < rototranslations.size(); ++j)
      {
        ros::Time publish_start = ros::Time::now();
        ROS_INFO_STREAM("Cluster correspondences: " << clustered_corrs[j].size());

        rec.rec_data_.percent_match_ = float(clustered_corrs[j].size()) / float(rec.rec_data_.model_scene_corrs.size());
        ROS_INFO_STREAM("Correspondence percentage: " << rec.rec_data_.percent_match_);

        pcl17::PointCloud<PointType> transformed_model;
        pcl17::transformPointCloud(rec.model_data_.points_, transformed_model, rototranslations[j]);
        pcl17::PointCloud<PointType> transformed_model_kp;
        pcl17::transformPointCloud(rec.model_data_.keypoints_, transformed_model_kp, rototranslations[j]);
        //ROS_WARN_STREAM("Model Points header frame: "<<rec.model_data_.points_.header.frame_id);

        sensor_msgs::PointCloud2 model_msg;
        pcl17::toROSMsg(transformed_model, model_msg);
        model_msg.header.frame_id = world_frame_;
        ROS_INFO_STREAM("Publishing " << model_msg.width << " points in " << model_msg.header.frame_id << " frame");
        model_pub_.publish(model_msg);

        //pcl17::PointCloud<PointType> transformed_model_kp;
        //pcl17::transformPointCloud(transformed_model_kp1, transformed_model_kp, Eigen::Affine3f(tfEigen));
        ROS_INFO_STREAM("Model keypoints cloud size "<<rec.model_data_.keypoints_.size());
        sensor_msgs::PointCloud2 model_kp_msg;
        pcl17::toROSMsg(transformed_model_kp, model_kp_msg);
        model_kp_msg.header.frame_id = world_frame_;
        model_kp_pub_.publish(model_kp_msg);

        geometry_msgs::Pose model_pose_msg;
        //POSE OF GENERATED TRAINING MODEL
        //ROS_INFO_STREAM("About to set pose_data to poses["<<best_match_<<"]...");
        //Eigen::Matrix4f mod_matrix = poses_[best_match_];
        std::string model_filename = rec.model_data_.source_name_;
        model_filename.erase(model_filename.end() - 4, model_filename.end());
        std::size_t found;
        std::string model_label = model_filename;
        found = model_label.find_last_of("_");
        std::string model_index = model_label.substr(found + 1);
        //ROS_INFO_STREAM("Model indicated/labeled as "<< model_index);
        std::string csv_filename = "pump_" + model_index + ".csv";
        std::string dir_csv = pose_path_ + csv_filename;
        ROS_INFO_STREAM("Reading csv file from directory: "<<dir_csv);
        std::ifstream input_pose_file;
        input_pose_file.open(dir_csv.c_str());
        std::string data[9];
        //double mat_data[9];
        std::vector<double> mat_data;
        for (int i = 0; i < 6; i++)
        {
          getline(input_pose_file, data[i], ',');
          //do the following to discard the space after the comma
          //getline( input_pose_file, std::string(), ' ' );

          mat_data.push_back(std::atof(data[i].c_str()));
        }
        input_pose_file.close();

        Eigen::Matrix4f mod_matrix;

        tf::Quaternion tf_quater;
        tf::Matrix3x3 tf_mod_matrix;
        tf_mod_matrix.setRPY(mat_data[0], mat_data[1], mat_data[2]);
        tf_mod_matrix.getRotation(tf_quater);
        double roll, pitch, yaw;
        tf_mod_matrix.getRPY(roll, pitch, yaw);
        tf::Vector3 tf_transl;
        tf_transl.setValue(mat_data[3], mat_data[4], mat_data[5]);

        /*Eigen::Vector3f eigen_vect = mod_matrix.block<3, 1>(0, 3);
         Eigen::Matrix3f model_rot = mod_matrix.block<3, 3>(0, 0);
         tf::Quaternion tf_quater;
         tf::Matrix3x3 tf_mod_matrix;
         tf_mod_matrix.setValue(model_rot(0, 0), model_rot(0, 1), model_rot(0, 2), model_rot(1, 0), model_rot(1, 1),
         model_rot(1, 2), model_rot(2, 0), model_rot(2, 1), model_rot(2, 2));
         tf_mod_matrix.getRotation(tf_quater);
         tf::Vector3 tf_transl;
         tf_transl.setValue(eigen_vect(0), eigen_vect(1), eigen_vect(2));*/
        tf::Transform tf_model;
        tf_model.setRotation(tf_quater);
        tf_model.setOrigin(tf_transl);
        ROS_INFO_STREAM("Model origin: "<< tf_transl.x()<<", " <<tf_transl.y()<<", "<<tf_transl.z());
        ROS_INFO_STREAM("Model roll, pitch, yaw: "<< roll <<", " <<pitch<<", "<<yaw);

        //POSE OF ROTOTRANSLATION

        Eigen::Matrix4d roto_double(rototranslations[j].cast<double>());
          Eigen::Affine3d affine(roto_double);
          tf::Transform tf_rototrans;
          tf::TransformEigenToTF(affine, tf_rototrans);

        /*Eigen::Matrix3f rotation = rototranslations[j].block<3, 3>(0, 0);
        Eigen::Vector3f translation = rototranslations[j].block<3, 1>(0, 3);
        ROS_INFO_STREAM("ROTOTRANS Eigen translation (x, y, z): ");


        tf::Matrix3x3 tf_matrix;
        tf_matrix.setValue(rotation(0, 0), rotation(0, 1), rotation(0, 2),
        		           rotation(1, 0), rotation(1, 1), rotation(1, 2),
        		           rotation(2, 0), rotation(2, 1), rotation(2, 2));
        tf::Vector3 tf_translation;
        tf_translation.setValue(translation(0), translation(1), translation(2));

        geometry_msgs::Transform trans;
        tf::Quaternion tf_quat;
        tf_matrix.getRotation(tf_quat);

        tf::Transform tf_rototrans;
        tf_rototrans.setRotation(tf_quat);
        tf_rototrans.setOrigin(tf_translation);
        ROS_INFO_STREAM("Rototranslation origin: "<< tf_translation.x()<<", " <<tf_translation.y()<<", "<<tf_translation.z());
*/
        //Part pose in passed in frame (world_frame)
        part_pose_ = tf_rototrans * tf_model;

        tf::Vector3 part_pose_xy = part_pose_.getOrigin();
        ROS_INFO_STREAM("Part pose origin at (x, y, z): "<<part_pose_xy.x()<<", "<<part_pose_xy.y()<<
                		", "<<part_pose_xy.z());
        //CHECK POSITION TO SEE IF OFF TABLE
        if (part_pose_xy.x()<-1.2 || part_pose_xy.x()>0 || part_pose_xy.y()>1.2 || part_pose_xy.y()<-0.2)
        {
        	ROS_WARN_STREAM("Part pose origin at (x, y, z): "<<part_pose_xy.x()<<", "<<part_pose_xy.y()<<
        		", "<<part_pose_xy.z());
        	part_pose_.setIdentity();
        	part_pose_= tf_model * tf_rototrans;
        	tf::Vector3 part_pose_xyn = part_pose_.getOrigin();
        	ROS_WARN_STREAM("Part pose origin at (x, y, z): "<<part_pose_xyn.x()<<", "<<part_pose_xyn.y()<<
        	        		", "<<part_pose_xyn.z());
        	part_pose_.setIdentity();

        }

        //CHECK ANGLE OF POSE TO SEE IF CLOSE TO VERTICAL
        tf::Vector3 vect(0.0f, 0.0f, -1.0f);
        tf::Matrix3x3 original_rot;
        original_rot = part_pose_.getBasis();
        tf::Vector3 z_vector = original_rot.getColumn(2);

        float angle_off = z_vector.angle(vect);
        ROS_INFO_STREAM(
            "Angle between z component of approach vector and z normal to table: " << angle_off*180/3.14159);

        //Ensure that pose is "facing down" such that robot approaches correctly
        int diff = (abs(angle_off * 180 / 3.14159)) % 180;
        int diff_from_vert;
        if (diff > 90)
        {
          diff_from_vert = abs(diff - 180);

        }
        else
        {
          diff_from_vert = diff;
        }

        ROS_INFO_STREAM("Using modulus and logic, diff between z and world: "<<diff_from_vert);

        //part pose with tool offset
        gripper_pose_.setOrigin(tf::Vector3(part_pose_xy.x(), part_pose_xy.y(), 0.080));				//part_pose_xy.z()
        tf::Quaternion grip_q;
        //tf::Quaternion part_q = part_pose_.getRotation();
        tf::Matrix3x3 part_r, grip_r;
        part_r = part_pose_.getBasis();
        tf::Vector3 x_v_p = part_r.getColumn(0);
        ROS_INFO_STREAM(
            "X vector for part to include in gripper orientation: "<< x_v_p.x() <<", "<<x_v_p.y()<<", "<< x_v_p.z());
        tf::Vector3 grip_z_v(0, 0, -1.0f);
        //tf::Vector3 dot_x=
        tf::Vector3 x_proj = x_v_p - tf::tfDot(x_v_p, grip_z_v) * grip_z_v;
        ROS_INFO_STREAM("X vector projection: "<< x_proj.getX() <<", "<<x_proj.getY()<<", "<< x_proj.getZ());
        tf::Vector3 grip_y_v = tf::tfCross(x_proj, grip_z_v);
        ROS_INFO_STREAM("Y vector from cross: "<< grip_y_v.getX() <<", "<<grip_y_v.getY()<<", "<< grip_y_v.getZ());
        grip_r.setValue(x_proj.getX(), grip_y_v.getX(), 0, x_proj.getY(), grip_y_v.getY(), 0, x_proj.getZ(),
                        grip_y_v.getZ(), -1.0f);
        double gripper_yaw, gripper_pitch, gripper_roll;
        grip_r.getEulerYPR(gripper_yaw, gripper_pitch, gripper_roll);
        //grip_r.setValue(x_v_p.x(), 0, 0, x_v_p.y(), 1, 0, x_v_p.z(), 0, -1.0f);
        //tf::Vector3 x_v_p=part_r.getRow(0);
        //grip_r.setValue(x_v_p.x(), x_v_p.y(), x_v_p.z(), 0, 1, 0, 0, 0, -1.0f);
        //grip_r.getRotation(grip_q);
        grip_q.setRPY(gripper_roll, gripper_pitch, gripper_yaw);
        //gripper_pose_.setRotation(grip_q);
        //gripper_pose_.setBasis(grip_r);
        grip_q.setRPY(3.14159, -0, 0);
        gripper_pose_.setRotation(grip_q);

        tf::Transform tf_trans_ee;
        tf_trans_ee.setIdentity();
        tf::Vector3 tf_transl_ee;
        tf_transl_ee.setValue(x_ee_offset_, y_ee_offset_, z_ee_offset_);
        tf_trans_ee.setOrigin(tf_transl_ee);

        gripper_pose_ = gripper_pose_ * tf_trans_ee;

        /*tf::Quaternion rot_x;
         rot_x.setValue(sin(3.14159/2), 0, 0, cos(3.14159/2));
         tf::Quaternion rot = gripper_pose_.getRotation();
         tf::Quaternion rotated_z=rot*rot_x;
         gripper_pose_.setRotation(rotated_z);*/

        //PUBLISH
        tf::poseTFToMsg(part_pose_, model_pose_msg);
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.pose.orientation = model_pose_msg.orientation;
        pose_msg.pose.position = model_pose_msg.position;
        pose_msg.header.frame_id = world_frame_;

        pose_pub_.publish(pose_msg);

        ros::Time publish_finish = ros::Time::now();
        ros::Duration publish_total = publish_finish - publish_start;
        ROS_INFO_STREAM("Calculating and publishing cloud and pose took "<< publish_total<<" s");

        //CHECK ANGLE OF POSE TO SEE IF CLOSE TO VERTICAL
        tf::Vector3 world_x_vect(1.0f, 0.0f, 0.0f);
        //tf::Matrix3x3 part_rot = part_pose_.getBasis();
        //tf::Vector3 part_x_vect = part_rot.getColumn(0);
        tf::Matrix3x3 grip_rot_check = gripper_pose_.getBasis();
        tf::Vector3 grip_x_vect = grip_rot_check.getColumn(0);

        //part_rot_ = part_x_vect.angle(world_x_vect);
        part_rot_=0;
        ROS_INFO_STREAM(
            "Angle between x component of gripper pose and x in world frame: " << (grip_x_vect.angle(world_x_vect))*180/3.14159);
        ROS_INFO_STREAM("Angle between x component of part pose and x in world frame: " << part_rot_*180/3.14159);

        if (diff_from_vert < 15)
        {
        	/*
            //part pose with tool offset
            gripper_pose_.setOrigin(tf::Vector3(part_pose_xy.x(), part_pose_xy.y(), part_pose_xy.z()));				//part_pose_xy.z()
            tf::Quaternion grip_q;
            //tf::Quaternion part_q = part_pose_.getRotation();
            tf::Matrix3x3 part_r, grip_r;
            part_r = part_pose_.getBasis();
            tf::Vector3 x_p = part_r.getColumn(0);
            tf::Vector3 y_p = part_r.getColumn(1);
            tf::Vector3 z_p = part_r.getColumn(2);
            //double part_r, part_p, part_y;
            //part_r.getEulerYPR(part_y, part_p, part_r);
            float downward = z_p.angle(vect);
            if (downward<0.262) // 11degrees
            {
            	gripper_pose_=part_pose_;
            	ROS_INFO_STREAM("Z vector grip: "<<z_p.x()<<", "<<z_p.y()<<", "<<z_p.z());
            }
            else
            {
            	tf::Quaternion interim_q, part_q;
            	part_q=part_pose_.getRotation();
            	interim_q.setRotation(x_p, 3.14159265);
            	grip_q=interim_q*part_q;
            	gripper_pose_.setRotation(grip_q);
            	grip_r=gripper_pose_.getBasis();
                tf::Vector3 z_g = grip_r.getColumn(2);
            	ROS_INFO_STREAM("Z vector grip: "<<z_g.x()<<", "<<z_g.y()<<", "<<z_g.z());
            }

            tf::Transform tf_trans_ee;
            tf_trans_ee.setIdentity();
            tf::Vector3 tf_transl_ee;
            tf_transl_ee.setValue(x_ee_offset_, y_ee_offset_, z_ee_offset_);
            tf_trans_ee.setOrigin(tf_transl_ee);

            gripper_pose_ = gripper_pose_ * tf_trans_ee;*/
          geometry_msgs::Pose grip_pose_msg;
          tf::poseTFToMsg(gripper_pose_, grip_pose_msg);
          geometry_msgs::PoseStamped grasp_pose_msg;
          grasp_pose_msg.pose.orientation = grip_pose_msg.orientation;
          grasp_pose_msg.pose.position = grip_pose_msg.position;
          grasp_pose_msg.header.frame_id = world_frame_;

          grip_pose_pub_.publish(grasp_pose_msg);
          success = true;
        }

        else
        {
          success = false;
        }
      }
    }
    ROS_INFO_STREAM("========================================================");
    ros::Timer myTime;
    ros::Duration(2.0).sleep();
    return success;
  }

  void spin()
  {

    ros::Rate loop_rate(0.05);

    const std::string FRAME_ID = "ur5_arm_kinect_rgb_optical_frame";
    std::string topic = nh_.resolveName(topic_);
    //std::string topic = nh_.resolveName("avg_filtered_cloud");//avg_filtered_cloud
    //sensor_msgs::PointCloud2::ConstPtr recent_cloud (new sensor_msgs::PointCloud2);
    pcl17::PointCloud<PointType> cloud;
    pcl17::PointCloud<PointType>::Ptr Cloud;
    std::vector<pcl17::PointCloud<PointType> > clouds;
    pcl17::PointCloud<PointType> xf_cloud;
    pcl17::PointCloud<PointType> yf_cloud;
    pcl17::PointCloud<PointType>::Ptr mls_filtered_cloud;
    pcl17::PointCloud<PointType>::Ptr sor_filtered_cloud;
    pcl17::PointCloud<PointType>::Ptr copy_cloud;
    //industrial_pcl_filters::ConcatenateMLS<PointType > concat_mls_filter;

    //int sample_number = 0;

    for (int k = 0; k < 50; k++)
    {
      if (ros::ok())
      {
        ROS_INFO_STREAM("Beginning of spin loop");

        //for (int j=0; j<num_images_; j++)
        //{
        //PRE_FILTERING OF INPUT CLOUD
        cloud.points.clear();
        ROS_INFO("Cloud service called; waiting for a point_cloud2 on topic %s", topic.c_str());
        sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic,
                                                                                                               nh_);
        //Passthrough filters
        pcl17::fromROSMsg(*recent_cloud, cloud);
        pcl17::PointCloud<PointType>::Ptr cloud_ptr(new pcl17::PointCloud<PointType>(cloud));
        pcl17::PassThrough<pcl17::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud_ptr);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(x_filter_min_, x_filter_max_);
        pass_x.filter(xf_cloud);

        pcl17::PointCloud<PointType>::Ptr f_cloud_ptr(new pcl17::PointCloud<PointType>(xf_cloud));
        pcl17::PassThrough<pcl17::PointXYZ> pass_y;
        pass_y.setInputCloud(f_cloud_ptr);
        pass_y.setFilterFieldName("z");
        pass_y.setFilterLimits(y_filter_min_, y_filter_max_);
        pass_y.filter(yf_cloud);

        clouds.push_back(yf_cloud);
        //}

        //concat_mls_filter.setInputCloud(Cloud);
        //concat_mls_filter.setInputClouds(clouds);
        //concat_mls_filter.filter(*mls_filtered_cloud);
        clouds.clear();

        //pcl17::copyPointCloud(*mls_filtered_cloud, *copy_cloud);

        //pcl17::PointCloud<PointType> obj_points;
        //pcl17::fromROSMsg(*recent_cloud, obj_points);

        sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
        pcl17::toROSMsg(yf_cloud, *pc2_cloud);
        pc2_cloud->header.frame_id = FRAME_ID;
        pc2_cloud->header.stamp = ros::Time::now();
        object_pub_.publish(pc2_cloud);

        if (yf_cloud.size() != 0)
        {
          ros::Time rec_total_start = ros::Time::now();
          sample_number_ = sample_number_ + 1;

          recognize(yf_cloud);

          ros::Time rec_total_finish = ros::Time::now();
          ros::Duration rec_total = rec_total_finish - rec_total_start;
          ROS_INFO_STREAM("For sample number: "<<sample_number_<<" total recognition took "<< rec_total<<" s");
          //STATFILE_<<rec_total;
        }
        else
        {
          ROS_INFO_STREAM("Cloud from scene is empty for some reason... trying again..");
          continue;
        }
        /*
         std::string filename = model_path_ + "/mug_view1.pcd";
         pcl17::PointCloud<PointType> obj_points;

         ROS_INFO_STREAM("Loading cloud data from " << filename);
         if (pcl17::io::loadPCDFile(filename, obj_points) < 0)
         {
         ROS_ERROR_STREAM("Error loading model: " << filename);
         }
         ROS_INFO_STREAM( "Loaded model: mug_view1.pcd with " << obj_points.size() << " points");
         //object_pub_.publish(recent_cloud);
         //pcl17::fromROSMsg(*recent_cloud, obj_points);

         recognize(obj_points);
         */
        /*tabletop_object_detector::TabletopSegmentation segmentation;
         if (tabletop_seg_client_.call(segmentation))
         {
         ROS_INFO_STREAM("Found " << segmentation.response.clusters.size() << "clusters");

         if (segmentation.response.clusters.size() > 0)
         {
         sensor_msgs::PointCloud2::Ptr obj_points_msg(new sensor_msgs::PointCloud2());
         pcl17::PointCloud<PointType> obj_points;

         ROS_INFO("Found a cluster, publishing it");
         ROS_INFO_STREAM("Cluster size:" << segmentation.response.clusters[0].points.size());
         ROS_INFO_STREAM("Segmentation returned frame ID: " << segmentation.response.clusters[0].header.frame_id);
         segmentation.response.clusters[0].header.frame_id = FRAME_ID;
         sensor_msgs::convertPointCloudToPointCloud2(segmentation.response.clusters[0], *obj_points_msg);
         object_pub_.publish(obj_points_msg);
         pcl17::fromROSMsg(*obj_points_msg, obj_points);

         recognize(obj_points);
         //loop_rate.sleep();
         }
         }*/
      }
    }
    return;
  }

  bool rec_CB(mantis_perception::mantis_recognition::Request &main_request,
              mantis_perception::mantis_recognition::Response &main_response)
  {
    ROS_INFO_STREAM("Recognition service called");

    if (test_mode_)
    {
      ROS_WARN_STREAM("Test mode enabled, returning fixed grasp pose");
      tf::Transform test_pose;
      tf::Quaternion test_quat;
      tf::Vector3 test_origin(-0.857901, 0.427257, 0.066);
      test_quat.setRPY(3.14159, -0, -1.86432);
      test_pose.setRotation(test_quat);
      test_pose.setOrigin(test_origin);

      geometry_msgs::PoseStamped test_gm_pose;
      geometry_msgs::Transform test_gm_trans;
      tf::transformTFToMsg(test_pose, test_gm_trans);
      test_gm_pose.pose.orientation = test_gm_trans.rotation; //pose.orientation x y z w & pose.position x y z & header frame_id stamp
      test_gm_pose.pose.position.x = test_gm_trans.translation.x; //rotation x y z w & translation x y z
      test_gm_pose.pose.position.y = test_gm_trans.translation.y;
      test_gm_pose.pose.position.z = test_gm_trans.translation.z;
      test_gm_pose.header.stamp = ros::Time::now();
      test_gm_pose.header.frame_id = world_frame_;

      grip_pose_pub_.publish(test_gm_pose);

      main_response.pick_poses.push_back(test_gm_pose);
      main_response.pose.x = test_gm_trans.translation.x;
      main_response.pose.y = test_gm_trans.translation.y;
      main_response.pose.z = test_gm_trans.translation.z;
      main_response.pose.rotation = 3.14159;
    }
    else
    {
      bool acceptable_pose;
      do
      {

        //const std::string FRAME_ID = "kinect_rgb_optical_frame";
        std::string topic = nh_.resolveName(topic_);
        //std::string topic = nh_.resolveName("avg_filtered_cloud");//avg_filtered_cloud
        pcl17::PointCloud<PointType> cloud;
        pcl17::PointCloud<PointType> xf_cloud;
        pcl17::PointCloud<PointType> yf_cloud;
        pcl17::PointCloud<PointType> zf_cloud;

        ROS_INFO("Cloud service called; waiting for a point_cloud2 on topic %s", topic.c_str());
        sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(topic,
                                                                                                               nh_);
        /*
         std::string filename = model_path_ + "/test_scene.pcd";
         pcl17::PointCloud<PointType> obj_points;

         ROS_INFO_STREAM("Loading cloud data from " << filename);
         if (pcl17::io::loadPCDFile(filename, obj_points) < 0)
         {
         ROS_ERROR_STREAM("Error loading model: " << filename);
         continue;
         }
         ROS_INFO_STREAM( "Loaded model: mug_view1.pcd with " << obj_points.size() << " points");*/
        //object_pub_.publish(recent_cloud);
        //pcl17::fromROSMsg(*recent_cloud, obj_points);
        sensor_msgs::PointCloud old_cloud;
        sensor_msgs::PointCloud2 transformed_cloud;
        std::string processing_frame = world_frame_;

        pcl17::fromROSMsg(*recent_cloud, cloud);
        //cloud = obj_points;
        tf::TransformListener tf_listener;
        tf::StampedTransform sceneTf;
        sceneTf.setIdentity();
        //std::string clusterFrameId = "/kinect_rgb_optical_frame";
        std::string clusterFrameId = recent_cloud->header.frame_id;
        ROS_INFO_STREAM("Cloud passed to recognition service with frame id: "<<clusterFrameId);
        try
        {
          tf_listener.waitForTransform(processing_frame, clusterFrameId, ros::Time::now(), ros::Duration(1.0));
          tf_listener.lookupTransform(processing_frame, clusterFrameId, ros::Time(0), sceneTf);
          ROS_INFO_STREAM(
              "Successfully received sceneTF on tf_listener for "<< clusterFrameId << " to "<< processing_frame);
        }
        catch (tf::TransformException ex)
        {
          ROS_ERROR(
              "%s", std::string("Failed to resolve transform from " + processing_frame + " to " + clusterFrameId + " \n\t\t" + " tf error msg: " + ex.what()).c_str());
          ROS_WARN("%s", std::string("Will use Identity as cluster transform").c_str());
          sceneTf.setData(tf::Transform::getIdentity());
          return false;
        }

        pcl17::PointCloud<PointType> transformed_scene;
        Eigen::Affine3d tfEigen_scene;
        tf::TransformTFToEigen(sceneTf, tfEigen_scene);				//clusterTf from ClusterFrame to WorldFrame
        pcl17::transformPointCloud(cloud, transformed_scene, Eigen::Affine3f(tfEigen_scene));
        ROS_INFO_STREAM("Successfully tranformed scene cloud to new frame");

        //Passthrough filters
        pcl17::PointCloud<PointType>::Ptr cloud_ptr(new pcl17::PointCloud<PointType>(transformed_scene));
        pcl17::PassThrough<pcl17::PointXYZ> pass_x;
        pass_x.setInputCloud(cloud_ptr);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(x_filter_min_, x_filter_max_);
        pass_x.filter(xf_cloud);
        ROS_INFO_STREAM("Success: passthrough x");

        pcl17::PointCloud<PointType>::Ptr f_cloud_ptr(new pcl17::PointCloud<PointType>(xf_cloud));
        pcl17::PassThrough<pcl17::PointXYZ> pass_y;
        pass_y.setInputCloud(f_cloud_ptr);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(y_filter_min_, y_filter_max_);
        pass_y.filter(yf_cloud);
        ROS_INFO_STREAM("Success: passthrough y");

        pcl17::PointCloud<PointType>::Ptr fil_cloud_ptr(new pcl17::PointCloud<PointType>(yf_cloud));
        pcl17::PassThrough<pcl17::PointXYZ> pass_z;
        pass_z.setInputCloud(fil_cloud_ptr);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(z_filter_min_, z_filter_max_);
        pass_z.filter(zf_cloud);
        ROS_INFO_STREAM("Success: passthrough z");

        pcl17::StatisticalOutlierRemoval<pcl17::PointXYZI> sor;
        pcl17::PointCloud<pcl17::PointXYZI>::Ptr pre_cloud(new pcl17::PointCloud<pcl17::PointXYZI>);
        pcl17::PointCloud<pcl17::PointXYZI> filtered_cloud;
        pcl17::PointCloud<pcl17::PointXYZ> f_cloud;
        pcl17::copyPointCloud(zf_cloud, *pre_cloud);
        sor.setInputCloud(pre_cloud);
        sor.setMeanK(2);
        sor.setStddevMulThresh(1.0);
        sor.filter(filtered_cloud);
        pcl17::copyPointCloud(filtered_cloud, f_cloud);

        ROS_INFO_STREAM("Cloud processed through passthrough filters");
        sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
        pcl17::toROSMsg(f_cloud, *pc2_cloud);
        pc2_cloud->header.frame_id = world_frame_;
        pc2_cloud->header.stamp = ros::Time::now();
        object_pub_.publish(pc2_cloud);
        ROS_INFO_STREAM("scene cloud published");

        if (filtered_cloud.size() != 0)
        {
          ros::Time rec_total_start = ros::Time::now();

          //if recognize returns true, continue
          ROS_INFO_STREAM("Scene cloud has "<<f_cloud.points.size());
          acceptable_pose = recognize(f_cloud);
          //acceptable_pose = recognize(yf_cloud);
          //acceptable_pose = recognize(transformed_scene);

          if (acceptable_pose)
          {
            sample_number_++;
            main_response.label = "pump";
            geometry_msgs::PoseStamped gm_pose;
            geometry_msgs::Transform gm_trans;
            tf::transformTFToMsg(gripper_pose_, gm_trans);
            gm_pose.pose.orientation = gm_trans.rotation; //pose.orientation x y z w & pose.position x y z & header frame_id stamp
            gm_pose.pose.position.x = gm_trans.translation.x; //rotation x y z w & translation x y z
            gm_pose.pose.position.y = gm_trans.translation.y;
            gm_pose.pose.position.z = gm_trans.translation.z;
            gm_pose.header.stamp = ros::Time::now();

            main_response.pick_poses.push_back(gm_pose);
            main_response.pose.x = gm_trans.translation.x;
            main_response.pose.y = gm_trans.translation.y;
            main_response.pose.z = gm_trans.translation.z;
            main_response.pose.rotation = part_rot_;

            tf::Quaternion pose_rot_forfile = part_pose_.getRotation();
            double roll_file, pitch_file, yaw_file;
            tf::Matrix3x3 matforfile;
            matforfile.setRotation(pose_rot_forfile);
            matforfile.getRPY(roll_file, pitch_file, yaw_file);

            STATFILE_ << roll_file << ',';
            STATFILE_ << pitch_file << ',';
            STATFILE_ << yaw_file << ',';
            STATFILE_ << part_pose_.getOrigin().x() << ',';
            STATFILE_ << part_pose_.getOrigin().y() << ',';
            STATFILE_ << part_pose_.getOrigin().z() << ',';

            ROS_INFO_STREAM("Sample Number: "<<sample_number_);
            STATFILE_ << sample_number_ << endl;
          }
          ros::Time rec_total_finish = ros::Time::now();
          ros::Duration rec_total = rec_total_finish - rec_total_start;
          ROS_INFO_STREAM("Total recognition took "<< rec_total<<" s");

          //else, if recognize returns false, start over...
        }
        else
        {
          ROS_ERROR_STREAM("Cloud from scene is empty for some reason... ");
          //return false;
        }

      } while (acceptable_pose == false);
    }
    return true;
  }

  void init_srv()
  {
    ROS_INFO_STREAM("Recognition Service initialized and waiting to be called");
    recognition_server_ = nh_.advertiseService("/mantis_object_recognition", &RecognitionNode::rec_CB, this);
    //sample_number_=0;
    ros::spin();
  }
};
//end class RecognitionNode

int main(int argc, char *argv[])
{

  // Copied from Robot Navigator
  ros::init(argc, argv, "recognition_node");
  ros::NodeHandle nh;

  RecognitionNode rec(nh);

  rec.init();
  //rec.spin();
  rec.init_srv();

  return 0;
}
;

