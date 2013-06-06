/*
 * process_faro_data.cpp
 *
 *  Created on: Apr 30, 2013
 *      Author: cgomez
 */

#include <ros/ros.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

ros::Publisher faro1_pose_pub;
ros::Publisher rec1_pose_pub;
ros::Publisher rec2_pose_pub;
ros::Publisher faro2_pose_pub;
ros::Publisher vis_pub;

std::string filename_;
std::string world_frame_;
bool need_convert_faro_base_;
std::string faro_input_file_;
std::string rec1_input_file_;
std::string rec2_input_file_;
std::string output_file_;
visualization_msgs::MarkerArray markers_;

std::vector<double> faro_data_;
std::vector<double> rec1_data_;
std::vector<double> rec2_data_;

void init(ros::NodeHandle nh_, ros::NodeHandle priv_nh_)
{
  priv_nh_.getParam("filename", filename_);
  priv_nh_.getParam("world_frame", world_frame_);
  priv_nh_.getParam("faro_input_file", faro_input_file_);
  priv_nh_.getParam("rec1_input_file", rec1_input_file_);
  priv_nh_.getParam("rec2_input_file", rec2_input_file_);
  priv_nh_.getParam("need_conversion_faro_base", need_convert_faro_base_);
  priv_nh_.getParam("output_file", output_file_);

  faro1_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("model_measured_pose1", 1);
  faro2_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("model_measured_pose2", 1);
  rec1_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("model_rec1_pose", 1);
  rec2_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("model_rec2_pose", 1);
  vis_pub = nh_.advertise<visualization_msgs::MarkerArray>("points_marker", 0);

}

void readfarofile(std::string input_file)
{
  ROS_INFO_STREAM("Reading faro file from directory: "<<input_file);
  std::ifstream input_pose_file;
  input_pose_file.open(input_file.c_str());
  std::string data;
  std::string line;
  std::stringstream iss;
  while (getline(input_pose_file, line))
  {
    iss << line;
    while (getline(iss, data, ';'))
    {
      faro_data_.push_back(std::atof(data.c_str()));
    }
    iss.clear();
  }
  /*std::string data[16];
   for (int i = 0; i < 15; i++)
   {
   getline(input_pose_file, data[i], ';');
   //do the following to discard the space after the comma
   //getline( input_pose_file, std::string(), ' ' );

   faro_data_.push_back(std::atof(data[i].c_str()));
   }*/
  input_pose_file.close();
}

void readrec1file(std::string input_file)
{
  ROS_INFO_STREAM("Reading faro file from directory: "<<input_file);
  std::ifstream input_pose_file;
  input_pose_file.open(input_file.c_str());
  std::string data;
  std::string line;
  std::stringstream iss;
  while (getline(input_pose_file, line))
  {
    iss << line;
    while (getline(iss, data, ';'))
    {
      rec1_data_.push_back(std::atof(data.c_str()));
    }
    iss.clear();
  }
  input_pose_file.close();
}

void readrec2file(std::string input_file)
{
  ROS_INFO_STREAM("Reading faro file from directory: "<<input_file);
  std::ifstream input_pose_file;
  input_pose_file.open(input_file.c_str());
  std::string data;
  std::string line;
  std::stringstream iss;
  while (getline(input_pose_file, line))
  {
    iss << line;
    while (getline(iss, data, ';'))
    {
      rec2_data_.push_back(std::atof(data.c_str()));
    }
    iss.clear();
  }
  input_pose_file.close();
}

void publish_faro_transform(tf::Transform faro_base)
{
  static tf::TransformBroadcaster br;

  //tf::Transform faro_base;

  //base_at_table - Actual,Circle,-291.052543573765,-100.393272630778,213.8235
  tf::Vector3 faro_origin(213.8235, 291.052543573765, -100.393272630778);
  faro_origin = faro_origin / 1000;
  faro_origin.setZ(faro_origin.z() + 0.10);
  faro_base.setOrigin(faro_origin);

  //+X_Coordinate System 1.i;0.9495;+Y_Coordinate System 1.i;0.3139;+Z_Coordinate System 1.i;-0.0008;
  //+X_Coordinate System 1.j;0.0007;+Y_Coordinate System 1.j;0.0005;+Z_Coordinate System 1.j;1.0000;
  //+X_Coordinate System 1.k;0.3139;+Y_Coordinate System 1.k-0.9495;+Z_Coordinate System 1.k;0.0002;
  tf::Matrix3x3 faro_rot;
  //faro_rot.setValue(0.9495, 0.3139, -0.0008, 0.0007, 0.0005, 1.0000, 0.3139, -0.9495, 0.0002);
  faro_rot.setValue(0.9495, 0.0007, 0.3139, 0.3139, 0.0005, -0.9495, -0.0008, 1.0000, 0.0002);
  tf::Quaternion faro_quat;
  double roll, pitch, yaw;
  faro_rot.getEulerYPR(yaw, pitch, roll);
  faro_quat.setRPY(roll, pitch, yaw);

  faro_base.setRotation(faro_quat);

  br.sendTransform(tf::StampedTransform(faro_base, ros::Time::now(), world_frame_, "faro_base"));

}

void make_markers(tf::Vector3 point1, tf::Vector3 point2, tf::Vector3 point3, tf::Vector3 origin)
{
  visualization_msgs::Marker marker1a;
  marker1a.header.frame_id = world_frame_;
  marker1a.header.stamp = ros::Time();
  marker1a.id = 4;
  marker1a.type = visualization_msgs::Marker::SPHERE;
  marker1a.action = visualization_msgs::Marker::ADD;
  marker1a.pose.position.x = point1.x();
  marker1a.pose.position.y = point1.y();
  marker1a.pose.position.z = point1.z();
  marker1a.color.a = 1.0;
  marker1a.color.r = 0.0;
  marker1a.color.g = 1.0;
  marker1a.color.b = 0.0;
  marker1a.scale.x = 0.04;
  marker1a.scale.y = 0.04;
  marker1a.scale.z = 0.04;
  markers_.markers.push_back(marker1a);
  visualization_msgs::Marker marker2a;
  marker2a.header.frame_id = world_frame_;
  marker2a.header.stamp = ros::Time();
  marker2a.id = 5;
  marker2a.type = visualization_msgs::Marker::SPHERE;
  marker2a.action = visualization_msgs::Marker::ADD;
  marker2a.pose.position.x = point2.x();
  marker2a.pose.position.y = point2.y();
  marker2a.pose.position.z = point2.z();
  marker2a.color.a = 1.0;
  marker2a.color.r = 1.0;
  marker2a.color.g = 0.0;
  marker2a.color.b = 0.0;
  marker2a.scale.x = 0.04;
  marker2a.scale.y = 0.04;
  marker2a.scale.z = 0.04;
  markers_.markers.push_back(marker2a);
  visualization_msgs::Marker marker3a;
  marker3a.header.frame_id = world_frame_;
  marker3a.header.stamp = ros::Time();
  marker3a.id = 6;
  marker3a.type = visualization_msgs::Marker::SPHERE;
  marker3a.action = visualization_msgs::Marker::ADD;
  marker3a.pose.position.x = point3.x();
  marker3a.pose.position.y = point3.y();
  marker3a.pose.position.z = point3.z();
  marker3a.color.a = 1.0;
  marker3a.color.r = 0.0;
  marker3a.color.g = 0.0;
  marker3a.color.b = 1.0;
  marker3a.scale.x = 0.04;
  marker3a.scale.y = 0.04;
  marker3a.scale.z = 0.04;
  markers_.markers.push_back(marker3a);
  visualization_msgs::Marker marker2;
  marker2.header.frame_id = world_frame_;
  marker2.header.stamp = ros::Time();
  marker2.id = 2;
  marker2.type = visualization_msgs::Marker::SPHERE;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.position.x = origin.x();
  marker2.pose.position.y = origin.y();
  marker2.pose.position.z = origin.z();
  marker2.color.a = 1.0;
  marker2.color.r = 0.5;
  marker2.color.g = 0.0;
  marker2.color.b = 0.5;
  marker2.scale.x = 0.02;
  marker2.scale.y = 0.02;
  marker2.scale.z = 0.02;
  markers_.markers.push_back(marker2);
  //vis_pub.publish(markers_);
}

void make_markers2(tf::Vector3 point1, tf::Vector3 point2, tf::Vector3 point3, tf::Vector3 origin)
{
  visualization_msgs::Marker marker1a;
  marker1a.header.frame_id = world_frame_;
  marker1a.header.stamp = ros::Time();
  marker1a.id = 10;
  marker1a.type = visualization_msgs::Marker::SPHERE;
  marker1a.action = visualization_msgs::Marker::ADD;
  marker1a.pose.position.x = point1.x();
  marker1a.pose.position.y = point1.y();
  marker1a.pose.position.z = point1.z();
  marker1a.color.a = 1.0;
  marker1a.color.r = 0.0;
  marker1a.color.g = 1.0;
  marker1a.color.b = 0.0;
  marker1a.scale.x = 0.04;
  marker1a.scale.y = 0.04;
  marker1a.scale.z = 0.04;
  markers_.markers.push_back(marker1a);
  visualization_msgs::Marker marker2a;
  marker2a.header.frame_id = world_frame_;
  marker2a.header.stamp = ros::Time();
  marker2a.id = 11;
  marker2a.type = visualization_msgs::Marker::SPHERE;
  marker2a.action = visualization_msgs::Marker::ADD;
  marker2a.pose.position.x = point2.x();
  marker2a.pose.position.y = point2.y();
  marker2a.pose.position.z = point2.z();
  marker2a.color.a = 1.0;
  marker2a.color.r = 1.0;
  marker2a.color.g = 0.0;
  marker2a.color.b = 0.0;
  marker2a.scale.x = 0.04;
  marker2a.scale.y = 0.04;
  marker2a.scale.z = 0.04;
  markers_.markers.push_back(marker2a);
  visualization_msgs::Marker marker3a;
  marker3a.header.frame_id = world_frame_;
  marker3a.header.stamp = ros::Time();
  marker3a.id = 12;
  marker3a.type = visualization_msgs::Marker::SPHERE;
  marker3a.action = visualization_msgs::Marker::ADD;
  marker3a.pose.position.x = point3.x();
  marker3a.pose.position.y = point3.y();
  marker3a.pose.position.z = point3.z();
  marker3a.color.a = 1.0;
  marker3a.color.r = 0.0;
  marker3a.color.g = 0.0;
  marker3a.color.b = 1.0;
  marker3a.scale.x = 0.04;
  marker3a.scale.y = 0.04;
  marker3a.scale.z = 0.04;
  markers_.markers.push_back(marker3a);
  visualization_msgs::Marker marker2;
  marker2.header.frame_id = world_frame_;
  marker2.header.stamp = ros::Time();
  marker2.id = 13;
  marker2.type = visualization_msgs::Marker::SPHERE;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.position.x = origin.x();
  marker2.pose.position.y = origin.y();
  marker2.pose.position.z = origin.z();
  marker2.color.a = 1.0;
  marker2.color.r = 0.5;
  marker2.color.g = 0.0;
  marker2.color.b = 0.5;
  marker2.scale.x = 0.02;
  marker2.scale.y = 0.02;
  marker2.scale.z = 0.02;
  markers_.markers.push_back(marker2);
  //vis_pub.publish(markers_);
}

tf::Transform process_measured_points(tf::Vector3 point1, tf::Vector3 point2, tf::Vector3 point3)
{
  tf::Transform pose;
  //0.0546165, 0.030832, 0.0565557
  double part_x_offset, part_y_offset, part_z_offset;
  part_x_offset = 0.0546165;
  part_y_offset = 0.030832;
  part_z_offset = 0.0565557;
  tf::Vector3 part_offset(part_x_offset, part_y_offset, part_z_offset);
  /*part_x_offset = point1.x() - part_center.x();
   part_y_offset = point1.y() - part_center.y();
   part_z_offset = point1.z() - part_center.z();*/
  ROS_INFO_STREAM("Part offsets x, y, z: "<<part_x_offset<<", "<<part_y_offset<<", "<<part_z_offset);

  tf::Vector3 x_axis, y_axis, z_axis;
  x_axis = point2 - point3;
  //x_axis = point3-point2;
  ROS_INFO_STREAM("X vector for part: "<< x_axis.x() <<", "<<x_axis.y()<<", "<< x_axis.z());
  y_axis = point1 - point2;
  //y_axis=point1-point2;
  z_axis = tf::tfCross(x_axis, y_axis);
  x_axis.normalize();
  ROS_INFO_STREAM("Normalized X vector for part: "<< x_axis.x() <<", "<<x_axis.y()<<", "<< x_axis.z());
  y_axis.normalize();
  z_axis.normalize();

  tf::Matrix3x3 part_o;
  part_o.setValue(x_axis.getX(), y_axis.getX(), z_axis.getX(), x_axis.getY(), y_axis.getY(), z_axis.getY(),
                  x_axis.getZ(), y_axis.getZ(), z_axis.getZ());
  double yaw, pitch, roll;
  part_o.getEulerYPR(yaw, pitch, roll);

  tf::Quaternion part_q;
  part_q.setRPY(roll, pitch, yaw);

  //tf::Transform part_pose;
  pose.setRotation(part_q);

  pose.setOrigin(tf::Vector3(0, 0, 0));

  tf::Vector3 offset_inpart = pose * part_offset;

  //double part_o_x, part_o_y, part_o_z;
  tf::Vector3 part_or;
  part_or = point1 - offset_inpart;
  //part_o_x = point1.x() + part_x_offset;
  //part_o_y = point1.y() + part_y_offset;
  //part_o_z = point1.z() + part_z_offset;
  tf::Vector3 origin(part_or.x(), part_or.y(), part_or.z());

  ROS_INFO_STREAM("Part origin " << origin.x()<<", "<< origin.y()<<", "<< origin.z());

  pose.setOrigin(origin);

  return pose;
}

void process()
{
  std::ofstream STATFILE(output_file_.c_str(), std::ios::out | std::ios::app);

  if (STATFILE.is_open())
      {
        STATFILE << "Error x, Error Y, Error Z, Error Translation, "
            "Error Yaw, Error Pitch, Error Roll, Error Rotation, "
            "Error Yaw2, Error Pitch2, Error Roll2, Error Rotation2  \n";
      }
      else
      {
        ROS_INFO_STREAM("Unable to open file " << output_file_);
      }
  tf::Transform faro_base;
  publish_faro_transform(faro_base);

  tf::Transform faro_to_base;
  //base_at_table - Actual,Circle,-291.052543573765,-100.393272630778,213.8235
  tf::Vector3 faro_origin(-291.052543573765, -100.393272630778, 213.8235);
  faro_origin = faro_origin / 1000;
  faro_to_base.setOrigin(faro_origin);

  //+X_Coordinate System 1.i;0.9495;+Y_Coordinate System 1.i;0.3139;+Z_Coordinate System 1.i;-0.0008;
  //+X_Coordinate System 1.j;0.0007;+Y_Coordinate System 1.j;0.0005;+Z_Coordinate System 1.j;1.0000;
  //+X_Coordinate System 1.k;0.3139;+Y_Coordinate System 1.k-0.9495;+Z_Coordinate System 1.k;0.0002;
  tf::Matrix3x3 faro_rot;
  faro_rot.setValue(0.9495, 0.3139, -0.0008, 0.0007, 0.0005, 1.0000, 0.3139, -0.9495, 0.0002);
  //faro_rot.setValue(0.9495, 0.0007, 0.3139, 0.3139, 0.0005, -0.9495, -0.0008, 1.0000, 0.0002);
  tf::Quaternion faro_quat;
  double f_roll, f_pitch, f_yaw;
  faro_rot.getEulerYPR(f_yaw, f_pitch, f_roll);
  faro_quat.setRPY(f_roll, f_pitch, f_yaw);

  faro_to_base.setRotation(faro_quat);

  /*tf::TransformListener tf_listener;
   tf::StampedTransform sceneTf; sceneTf.setIdentity();
   try
   {
   tf_listener.waitForTransform(world_frame_, "/faro_base", ros::Time::now(), ros::Duration(2.0));
   tf_listener.lookupTransform(world_frame_, "/faro_base", ros::Time(0), sceneTf);
   ROS_INFO_STREAM("Successfully received sceneTF on tf_listener for faro_base to "<< world_frame_);
   }
   catch (tf::TransformException ex)
   {
   ROS_ERROR(
   "%s", std::string("Failed to resolve transform from " + world_frame_ + " to /faro_base"+ " \n\t\t" + " tf error msg: " + ex.what()).c_str());
   ROS_WARN("%s", std::string("Will use Identity as cluster transform").c_str());
   sceneTf.setData(tf::Transform::getIdentity());
   //return false;
   }*/

  tf::Transform w_to_f;  //rosrun tf tf_echo /ur5_stand /faro_base
  w_to_f.setOrigin(tf::Vector3(0.214, 0.291, -0.000));
  w_to_f.setRotation(tf::Quaternion(0.698, 0.113, 0.112, 0.698));

  tf::Transform world_to_faro;
  world_to_faro.setOrigin(w_to_f.getOrigin());
  world_to_faro.setRotation(w_to_f.getRotation());

  readfarofile(faro_input_file_);
  readrec1file(rec1_input_file_);
  readrec2file(rec2_input_file_);

  //tf::Vector3 point1(-135.315323493412, 12.47042141, -481.577210615531);
  //tf::Vector3 point2(-151.885, 12.1727263408517, -424.921);
  //tf::Vector3 point3(-258.455427783646, 12.416743, -456.46538);
  for (int i = 0; i < 32; i = i++)
  {
    ROS_INFO_STREAM("Displaying results from Sample Number (N=): "<<i+1);
    //int fcount = i * 36;
    int fcount = i * 18;
    tf::Vector3 point1(faro_data_[fcount], faro_data_[fcount + 1], (faro_data_[fcount + 2] + 0.1));
    tf::Vector3 point2(faro_data_[fcount + 3], faro_data_[fcount + 4], (faro_data_[fcount + 5] + 0.1));
    tf::Vector3 point3(faro_data_[fcount + 6], faro_data_[fcount + 7], (faro_data_[fcount + 8] + 0.1));

    /*tf::Vector3 part_center(-196.6959292, -100.168336468461, -469.441058261334);
     part_center = part_center / 1000;
     tf::Vector3 part_center_world = world_to_faro * part_center;
     tf::Vector3 proper_center(part_center_world.x(), part_center_world.y(), (part_center_world.z() + 0.056));*/

    //convert from mm to m
    //point1 = point1 / 1000;
    //point2 = point2 / 1000;
    //point3 = point3 / 1000;
    tf::Vector3 point1_world, point2_world, point3_world;

    if (need_convert_faro_base_)
    {
      point1_world = world_to_faro * point1;
      point2_world = world_to_faro * point2;
      point3_world = world_to_faro * point3;
    }
    else
    {
      point1_world = point1;
      point2_world = point2;
      point3_world = point3;
    }

    tf::Transform faro_part_pose = process_measured_points(point1_world, point2_world, point3_world);
    ROS_INFO_STREAM("Point1 " << point1_world.x()<<", "<< point1_world.y()<<", "<< point1_world.z());
    ROS_INFO_STREAM("Point2 " << point2_world.x()<<", "<< point2_world.y()<<", "<< point2_world.z());
    ROS_INFO_STREAM("Point3 " << point3_world.x()<<", "<< point3_world.y()<<", "<< point3_world.z());
    /*visualization_msgs::Marker marker1;
     marker1.header.frame_id = world_frame_;
     marker1.header.stamp = ros::Time();
     marker1.id = 1;
     marker1.type = visualization_msgs::Marker::SPHERE;
     marker1.action = visualization_msgs::Marker::ADD;
     marker1.pose.position.x = part_center_world.x();
     marker1.pose.position.y = part_center_world.y();
     marker1.pose.position.z = part_center_world.z();
     marker1.color.a = 1.0;
     marker1.color.r = 0.5;
     marker1.color.g = 0.0;
     marker1.color.b = 0.5;
     marker1.scale.x = 0.05;
     marker1.scale.y = 0.05;
     marker1.scale.z = 0.05;
     markers_.markers.push_back(marker1);*/
    tf::Vector3 f1_origin = faro_part_pose.getOrigin();
    ROS_INFO_STREAM("Faro Part origin1 " << f1_origin.x()<<", "<< f1_origin.y()<<", "<< f1_origin.z());

    //publish markers
    make_markers(point1_world, point2_world, point3_world, f1_origin);
    //vis_pub.publish(markers_);

    //publish_pose faro position 1 pose
    geometry_msgs::Pose faro_pose_msg;
    tf::poseTFToMsg(faro_part_pose, faro_pose_msg);
    geometry_msgs::PoseStamped f_pose_msg;
    f_pose_msg.pose.orientation = faro_pose_msg.orientation;
    f_pose_msg.pose.position = faro_pose_msg.position;
    f_pose_msg.header.frame_id = world_frame_;
    ROS_INFO_STREAM("Header frame: "<<f_pose_msg.header.frame_id);
    f_pose_msg.header.stamp = ros::Time::now();
    faro1_pose_pub.publish(f_pose_msg);

    int r1count = i * 7;
    tf::Transform rec1_pose;
    tf::Quaternion rec1_quat;
    rec1_quat.setRPY(rec1_data_[r1count], rec1_data_[r1count + 1], rec1_data_[r1count + 2]);
    rec1_pose.setRotation(rec1_quat);
    rec1_pose.setOrigin(tf::Vector3(rec1_data_[r1count + 3], rec1_data_[r1count + 4], rec1_data_[r1count + 5]));
    tf::Vector3 rec1_origin = rec1_pose.getOrigin();
    ROS_INFO_STREAM("Origin rec1 " << rec1_origin.x()<<", "<< rec1_origin.y()<<", "<< rec1_origin.z());
    geometry_msgs::Pose rec1_pose_msg;
    //tf::poseTFToMsg(transformed_part_pose, part_pose_msg);
    tf::poseTFToMsg(rec1_pose, rec1_pose_msg);
    geometry_msgs::PoseStamped r1_pose_msg;
    r1_pose_msg.pose.orientation = rec1_pose_msg.orientation;
    r1_pose_msg.pose.position = rec1_pose_msg.position;
    r1_pose_msg.header.frame_id = "/workcell_frame";
    ROS_INFO_STREAM("Header frame: "<<r1_pose_msg.header.frame_id);
    r1_pose_msg.header.stamp = ros::Time::now();
    rec1_pose_pub.publish(r1_pose_msg);

    double err_x, err_y, err_z, err_trans;
    err_x=f1_origin.x()-0.238-rec1_origin.x();
    err_y=f1_origin.y()-rec1_origin.y();
    err_z=(f1_origin.z()+0.1)-rec1_origin.z();
    err_trans=std::sqrt(err_x*err_x + err_y*err_y + err_z*err_z);
    STATFILE << err_x << ',';
    STATFILE << err_y << ',';
    STATFILE << err_z << ',';
    STATFILE << err_trans << ',';
    //have faro_part_pose and rec1_pose, need R's
    tf::Matrix3x3 Rfaro_workcell, Rrec1_workcell, Rworkcell_rec1, Rfaro_rec1;
    Rfaro_workcell=faro_part_pose.getBasis();
    Rrec1_workcell=rec1_pose.getBasis();
    Rworkcell_rec1=Rrec1_workcell.transpose();
    //need Rfarorec1, have Rfaroworkcell and Rrec1workcell, need to calc Rfaroworcell*Rrec1workcell_transpose
    Rfaro_rec1=Rfaro_workcell*Rworkcell_rec1;
    double euler_yaw1, euler_pitch1, euler_roll1;
    Rfaro_rec1.getEulerZYX(euler_yaw1, euler_pitch1, euler_roll1);
    double R00 = Rfaro_rec1.getRow(0).getX();
    double R11 = Rfaro_rec1.getRow(1).getY();
    double R22 = Rfaro_rec1.getRow(2).getZ();
    double trace = R00+R11+R22;
    double err_rot = acos( (trace -1)/2);
    STATFILE << euler_yaw1 << ',';
    STATFILE << euler_pitch1 << ',';
    STATFILE << euler_roll1 << ',';
    STATFILE << err_rot << ',';

    //2ndary position

    //int f2count = i * 36 + 18;
    int f2count = i*18+9;
    tf::Vector3 point1a(faro_data_[f2count], faro_data_[f2count + 1], (faro_data_[f2count + 2] + 0.1));
    tf::Vector3 point2a(faro_data_[f2count + 3], faro_data_[f2count + 4], (faro_data_[f2count + 5] + 0.1));
    tf::Vector3 point3a(faro_data_[f2count + 6], faro_data_[f2count + 7], (faro_data_[f2count + 8] + 0.1));

    tf::Transform faro2_part_pose = process_measured_points(point1a, point2a, point3a);
    tf::Vector3 f2_origin = faro2_part_pose.getOrigin();

    //publish markers
    make_markers2(point1a, point2a, point3a, f2_origin);
    vis_pub.publish(markers_);

    //publish_pose
    geometry_msgs::Pose faro2_pose_msg;
    //tf::poseTFToMsg(transformed_part_pose, part_pose_msg);
    tf::poseTFToMsg(faro2_part_pose, faro2_pose_msg);
    geometry_msgs::PoseStamped f2_pose_msg;
    f2_pose_msg.pose.orientation = faro2_pose_msg.orientation;
    f2_pose_msg.pose.position = faro2_pose_msg.position;
    f2_pose_msg.header.frame_id = world_frame_;
    ROS_INFO_STREAM("Header frame: "<<f2_pose_msg.header.frame_id);
    f2_pose_msg.header.stamp = ros::Time::now();
    faro2_pose_pub.publish(f2_pose_msg);

    int r2count = i;
    tf::Transform rec2_pose;
    tf::Quaternion rec2_quat, rec2_intermediate, rec2q;
    rec2q.setRPY(1.57, 0, 1.57);
    rec2_intermediate.setRPY(0, 0, -rec2_data_[r2count]);
    rec2_quat=rec2q*rec2_intermediate;
    rec2_pose.setRotation(rec2_quat);
    rec2_pose.setOrigin(f2_origin);

    geometry_msgs::Pose rec2_pose_msg;
    //tf::poseTFToMsg(transformed_part_pose, part_pose_msg);
    tf::poseTFToMsg(rec2_pose, rec2_pose_msg);
    geometry_msgs::PoseStamped r2_pose_msg;
    r2_pose_msg.pose.orientation = rec2_pose_msg.orientation;
    r2_pose_msg.pose.position = rec2_pose_msg.position;
    r2_pose_msg.header.frame_id = world_frame_;
    ROS_INFO_STREAM("Header frame: "<<r2_pose_msg.header.frame_id);
    r2_pose_msg.header.stamp = ros::Time::now();
    rec2_pose_pub.publish(r2_pose_msg);

    //have faro2_part_pose and rec2_pose, need R's
    tf::Matrix3x3 Rfaro2_workcell, Rrec2_workcell, Rworkcell_rec2, Rfaro_rec2;
    Rfaro2_workcell=faro2_part_pose.getBasis();
    Rrec2_workcell=rec2_pose.getBasis();
    Rworkcell_rec2=Rrec2_workcell.transpose();
    //need Rfarorec1, have Rfaroworkcell and Rrec1workcell, need to calc Rfaroworcell*Rrec1workcell_transpose
    Rfaro_rec2=Rfaro2_workcell*Rworkcell_rec2;
    double euler_yaw2, euler_pitch2, euler_roll2;
    Rfaro_rec2.getEulerZYX(euler_yaw2, euler_pitch2, euler_roll2);
    double R00_2 = Rfaro_rec2.getRow(0).getX();
    double R11_2 = Rfaro_rec2.getRow(1).getY();
    double R22_2 = Rfaro_rec2.getRow(2).getZ();
    double trace2 = R00_2+R11_2+R22_2;
    double err_rot2 = acos( (trace2 -1)/2);
    STATFILE << euler_yaw2 << ',';
    STATFILE << euler_pitch2 << ',';
    STATFILE << euler_roll2 << ',';
    STATFILE << err_rot2 << ',';
    STATFILE <<std::endl;

    sleep(1);
  }
}

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "faro_data_procesing_node");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");
  //GCRecognitionNode rec(nh);
  //rec.init();
  //rec.spin();

  init(nh, priv_nh);
  //process();

  //while (ros::ok())
  //{
    process();
  //}

  //ros::spin();
  //return 0;
}
