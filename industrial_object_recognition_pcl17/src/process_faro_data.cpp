/*
 * process_faro_data.cpp
 *
 *  Created on: Apr 30, 2013
 *      Author: cgomez
 */

#include <ros/ros.h>
#include <math.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

ros::Publisher pose_pub;
ros::Publisher vis_pub;

std::string filename_;
std::string world_frame_;

void init(ros::NodeHandle nh_, ros::NodeHandle priv_nh_)
{
  priv_nh_.getParam("filename", filename_);
  priv_nh_.getParam("world_frame", world_frame_);

  pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("model_measured_pose", 1);
  vis_pub = nh_.advertise<visualization_msgs::MarkerArray>( "points_marker", 0 );
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

void process_measured_points(tf::Transform world_to_faro, tf::Vector3 part_center, tf::Vector3 point1, tf::Vector3 point2,
                             tf::Vector3 point3, tf::Transform part_pose)
{
  part_center = part_center / 1000;

    tf::Vector3 part_center_world=world_to_faro*part_center;

    //convert from mm to m
    ROS_INFO_STREAM("Point1 " << point1.x()<<", "<< point1.y()<<", "<< point1.z());
    point1 = point1 / 1000;
    ROS_INFO_STREAM("Point1/1000 " << point1.x()<<", "<< point1.y()<<", "<< point1.z());
    point2 = point2 / 1000;
    point3 = point3 / 1000;


    tf::Vector3 point1_world=world_to_faro*point1;
    tf::Vector3 point2_world=world_to_faro*point2;
    tf::Vector3 point3_world=world_to_faro*point3;

    //get x-dir and y-dir, then use cross to get z_axis
    double part_x_offset, part_y_offset, part_z_offset;
    part_x_offset=point1_world.x()-part_center_world.x();
    part_y_offset=point1_world.y()-part_center_world.y();
    part_z_offset=point1_world.z()-part_center_world.z();
    /*part_x_offset = point1.z() - part_center.z();
    part_y_offset = point1.x() - part_center.x();
    part_z_offset = point1.y() - part_center.y();*/

    tf::Vector3 x_axis, y_axis, z_axis;
    x_axis = point3_world - point2_world;
    ROS_INFO_STREAM("X vector for part: "<< x_axis.x() <<", "<<x_axis.y()<<", "<< x_axis.z());
    y_axis = point1_world - point2_world;
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
    part_pose.setRotation(part_q);

    double part_o_x, part_o_y, part_o_z;
    part_o_x = part_center.z() - part_x_offset;
    part_o_y = part_center.x() - part_y_offset;
    tf::Vector3 origin(part_o_x, part_o_y, part_z_offset);
    ROS_INFO_STREAM("Part origin " << origin.x()<<", "<< origin.y()<<", "<< origin.z());

    part_pose.setOrigin(origin);


}

void process()
{
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

  tf::Transform w_to_f;//rosrun tf tf_echo /ur5_stand /faro_base
  w_to_f.setOrigin(tf::Vector3(0.214, 0.291, -0.000));
  w_to_f.setRotation(tf::Quaternion(0.698, 0.113, 0.112, 0.698));
  //w_to_f.setOrigin(tf::Vector3(-0.294, 0.100, 0.209));
  //w_to_f.setRotation(tf::Quaternion(-0.698, -0.113, -0.112, 0.698));

  tf::Transform world_to_faro;
  world_to_faro.setOrigin(w_to_f.getOrigin());
  world_to_faro.setRotation(w_to_f.getRotation());

  tf::Vector3 part_center(-196.6959292, -100.168336468461, -469.441058261334);
  part_center = part_center / 1000;
  tf::Vector3 part_center_world=world_to_faro*part_center;
  tf::Vector3 proper_center(part_center_world.x(), part_center_world.y(), (part_center_world.z()+0.056));


  tf::Vector3 point1(-135.315323493412, 12.47042141, -481.577210615531);
  tf::Vector3 point2(-151.885, 12.1727263408517, -424.921);
  tf::Vector3 point3(-258.455427783646, 12.416743, -456.46538);

  tf::Transform part_pose;

  //process_measured_points(world_to_faro, part_center, point1, point2, point3, part_pose);

  visualization_msgs::MarkerArray markers;

  //convert from mm to m
  ROS_INFO_STREAM("Point1 " << point1.x()<<", "<< point1.y()<<", "<< point1.z());
  point1 = point1 / 1000;
  ROS_INFO_STREAM("Point1/1000 " << point1.x()<<", "<< point1.y()<<", "<< point1.z());
  point2 = point2 / 1000;
  point3 = point3 / 1000;
  tf::Vector3 point1_world=world_to_faro*point1;
  tf::Vector3 point2_world=world_to_faro*point2;
  tf::Vector3 point3_world=world_to_faro*point3;

  visualization_msgs::Marker marker1;
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
  markers.markers.push_back(marker1);
  visualization_msgs::Marker marker1a;
  marker1a.header.frame_id = world_frame_;
  marker1a.header.stamp = ros::Time();
  marker1a.id = 4;
  marker1a.type = visualization_msgs::Marker::SPHERE;
  marker1a.action = visualization_msgs::Marker::ADD;
  marker1a.pose.position.x = point1_world.x();
  marker1a.pose.position.y = point1_world.y();
  marker1a.pose.position.z = point1_world.z();
  marker1a.color.a = 1.0;
  marker1a.color.r = 0.0;
  marker1a.color.g = 1.0;
  marker1a.color.b = 0.0;
  marker1a.scale.x = 0.05;
  marker1a.scale.y = 0.05;
  marker1a.scale.z = 0.05;
  markers.markers.push_back(marker1a);
  visualization_msgs::Marker marker2a;
  marker2a.header.frame_id = world_frame_;
  marker2a.header.stamp = ros::Time();
  marker2a.id = 5;
  marker2a.type = visualization_msgs::Marker::SPHERE;
  marker2a.action = visualization_msgs::Marker::ADD;
  marker2a.pose.position.x = point2_world.x();
  marker2a.pose.position.y = point2_world.y();
  marker2a.pose.position.z = point2_world.z();
  marker2a.color.a = 1.0;
  marker2a.color.r = 1.0;
  marker2a.color.g = 0.0;
  marker2a.color.b = 0.0;
  marker2a.scale.x = 0.05;
  marker2a.scale.y = 0.05;
  marker2a.scale.z = 0.05;
  markers.markers.push_back(marker2a);
  visualization_msgs::Marker marker3a;
  marker3a.header.frame_id = world_frame_;
  marker3a.header.stamp = ros::Time();
  marker3a.id = 6;
  marker3a.type = visualization_msgs::Marker::SPHERE;
  marker3a.action = visualization_msgs::Marker::ADD;
  marker3a.pose.position.x = point3_world.x();
  marker3a.pose.position.y = point3_world.y();
  marker3a.pose.position.z = point3_world.z();
  marker3a.color.a = 1.0;
  marker3a.color.r = 0.0;
  marker3a.color.g = 0.0;
  marker3a.color.b = 1.0;
  marker3a.scale.x = 0.05;
  marker3a.scale.y = 0.05;
  marker3a.scale.z = 0.05;
  markers.markers.push_back(marker3a);


  //get x-dir and y-dir, then use cross to get z_axis
  double part_x_offset, part_y_offset, part_z_offset;
  part_x_offset=point1_world.x()-proper_center.x();
  part_y_offset=point1_world.y()-proper_center.y();
  part_z_offset=point1_world.z()-proper_center.z();
  //part_x_offset = point1.z() - part_center.z();
  //part_y_offset = point1.x() - part_center.x();
  //part_z_offset = point1.y() - part_center.y();

  tf::Vector3 x_axis, y_axis, z_axis;
  x_axis = point2_world - point3_world;
  //x_axis = point3-point2;
  ROS_INFO_STREAM("X vector for part: "<< x_axis.x() <<", "<<x_axis.y()<<", "<< x_axis.z());
  y_axis = point1_world - point2_world;
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
  part_pose.setRotation(part_q);

  double part_o_x, part_o_y, part_o_z;
  part_o_x = point1_world.x() - part_x_offset;
  part_o_y = point1_world.y() - part_y_offset;
  part_o_z = point1_world.z() - part_z_offset;
  tf::Vector3 origin(part_o_x, part_o_y, part_o_z);
  ROS_INFO_STREAM("Part origin " << origin.x()<<", "<< origin.y()<<", "<< origin.z());
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
  marker2.color.r = 0.0;
  marker2.color.g = 0.0;
  marker2.color.b = 1.0;
  marker2.scale.x = 0.05;
  marker2.scale.y = 0.05;
  marker2.scale.z = 0.05;
  markers.markers.push_back(marker2);
  vis_pub.publish( markers );

  part_pose.setOrigin(origin);

  //publish_pose
  geometry_msgs::Pose part_pose_msg;
  //tf::poseTFToMsg(transformed_part_pose, part_pose_msg);
  tf::poseTFToMsg(part_pose, part_pose_msg);
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.pose.orientation = part_pose_msg.orientation;
  pose_msg.pose.position = part_pose_msg.position;
  ROS_INFO_STREAM("Header frame: "<<world_frame_);
  pose_msg.header.frame_id = world_frame_;
  pose_msg.header.stamp = ros::Time::now();

  pose_pub.publish(pose_msg);
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

  while (ros::ok())
  {
    process();
  }

  ros::spin();
  //return 0;
}
