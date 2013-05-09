/*
 * generate_training_data.cpp
 *
 *  Created on: Apr 23, 2013
 *      Author: cgomez
 */

#include <ros/ros.h>
#include <iostream>
#include <pcl17/point_types.h>
#include <pcl17/point_cloud.h>
//#include <pcl17/io/vtk_lib_io.h>
#include <pcl17/io/pcd_io.h>
#include <pcl17/common/transforms.h>
#include <pcl17/visualization/pcl_visualizer.h>
#include <pcl17/console/print.h>
#include <pcl17/filters/voxel_grid.h>
#include <pcl17/filters/filter.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <iostream>
#include <pcl17/apps/render_views_tesselated_sphere.h>
#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "training_node");
  ros::NodeHandle nh;

  std::string ply_model_path_;
  std::string model_name_;
  double pos_x_;
  double pos_y_;
  double pos_z_;
  double view_x_;
  double view_y_;
  double view_z_;
  double up_x_;
  double up_y_;
  double up_z_;
  double radius_sphere_;
  int view_angle_;
  int resolution_;

  ROS_INFO_STREAM("Loading private parameters");
  ros::NodeHandle private_nh_("~");
  private_nh_.getParam("ply_model_path", ply_model_path_);
  private_nh_.getParam("model_name", model_name_);
  private_nh_.getParam("pos_x", pos_x_);
  private_nh_.getParam("pos_y", pos_y_);
  private_nh_.getParam("pos_z", pos_z_);
  private_nh_.getParam("view_x", view_x_);
  private_nh_.getParam("view_y", view_y_);
  private_nh_.getParam("view_z", view_z_);
  private_nh_.getParam("up_x", up_x_);
  private_nh_.getParam("up_y", up_y_);
  private_nh_.getParam("up_z", up_z_);
  private_nh_.getParam("radius_sphere", radius_sphere_);
  private_nh_.getParam("view_angle", view_angle_);
  private_nh_.getParam("resolution", resolution_);

  ROS_INFO_STREAM("finished loading private parameters");
  std::string filename = ply_model_path_ + "/" + model_name_;
  vtkSmartPointer<vtkPLYReader> readerQuery = vtkSmartPointer<vtkPLYReader>::New();
  vtkSmartPointer<vtkPolyDataMapper> poly_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  readerQuery->SetFileName(filename.c_str());
  vtkSmartPointer<vtkPolyData> polydata; // = readerQuery->GetOutput();
  //polydata->Update();

  //pcl17::visualization::PCLVisualizer vis("Visualizer");

  //vis.addModelFromPolyData(polydata, "mesh1", 0);
  poly_mapper->SetInputConnection(readerQuery->GetOutputPort());
  poly_mapper->Update();
  vtkSmartPointer<vtkPolyData> pdata = vtkSmartPointer<vtkPolyData>::New();
  pdata->Allocate(poly_mapper->GetInput());
  polydata = poly_mapper->GetInput();

  ROS_INFO_STREAM("polydata allocated");

  double CoM[3];
  vtkIdType npts_com = 0, *ptIds_com = NULL;
  vtkSmartPointer<vtkCellArray> cells_com = polydata->GetPolys();
  double center[3], p1_com[3], p2_com[3], p3_com[3], area_com, totalArea_com = 0;
  double comx = 0, comy = 0, comz = 0;
  for (cells_com->InitTraversal(); cells_com->GetNextCell(npts_com, ptIds_com);)
  {
    polydata->GetPoint(ptIds_com[0], p1_com);
    polydata->GetPoint(ptIds_com[1], p2_com);
    polydata->GetPoint(ptIds_com[2], p3_com);
    vtkTriangle::TriangleCenter(p1_com, p2_com, p3_com, center);
    area_com = vtkTriangle::TriangleArea(p1_com, p2_com, p3_com);
    comx += center[0] * area_com;
    comy += center[1] * area_com;
    comz += center[2] * area_com;
    totalArea_com += area_com;
  }

  CoM[0] = comx / totalArea_com;
  CoM[1] = comy / totalArea_com;
  CoM[2] = comz / totalArea_com;
  ROS_INFO_STREAM("Area computed");

  vtkSmartPointer<vtkTransform> trans_center = vtkSmartPointer<vtkTransform>::New();
  trans_center->Translate(-CoM[0], -CoM[1], -CoM[2]);
  vtkSmartPointer<vtkMatrix4x4> matrixCenter = trans_center->GetMatrix();

  vtkSmartPointer<vtkTransformFilter> trans_filter_center = vtkSmartPointer<vtkTransformFilter>::New();
  trans_filter_center->SetTransform(trans_center);
  trans_filter_center->SetInput(polydata);
  trans_filter_center->Update();

  vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(trans_filter_center->GetOutputPort());
  mapper->Update();
  ROS_INFO_STREAM("trans center, mapper update");
  //scale so it fits in the unit sphere!
  double bb[6];
  mapper->GetBounds(bb);
  double ms = (std::max)((std::fabs)(bb[0] - bb[1]),
                         (std::max)((std::fabs)(bb[2] - bb[3]), (std::fabs)(bb[4] - bb[5])));
  double max_side = pos_z_ / 2.0;
  double scale_factor = max_side / ms;

  vtkSmartPointer<vtkTransform> trans_scale = vtkSmartPointer<vtkTransform>::New();
  trans_scale->Scale(scale_factor, scale_factor, scale_factor);
  vtkSmartPointer<vtkMatrix4x4> matrixScale = trans_scale->GetMatrix();

  vtkSmartPointer<vtkTransformFilter> trans_filter_scale = vtkSmartPointer<vtkTransformFilter>::New();
  trans_filter_scale->SetTransform(trans_scale);
  trans_filter_scale->SetInputConnection(trans_filter_center->GetOutputPort());
  trans_filter_scale->Update();

  mapper->SetInputConnection(trans_filter_scale->GetOutputPort());
  mapper->Update();

  vtkSmartPointer<vtkCellArray> cells = mapper->GetInput()->GetPolys();
  vtkIdType npts = 0, *ptIds = NULL;

  double p1[3], p2[3], p3[3], area, totalArea = 0;
  for (cells->InitTraversal(); cells->GetNextCell(npts, ptIds);)
  {
    polydata->GetPoint(ptIds[0], p1);
    polydata->GetPoint(ptIds[1], p2);
    polydata->GetPoint(ptIds[2], p3);
    area = vtkTriangle::TriangleArea(p1, p2, p3);
    totalArea += area;
  }

  vtkSmartPointer<vtkPolyData> sphere = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
  //vtkSmartPointer<vtkCellArray> pts = vtkSmartPointer<vtkCellArray>::New();
  for (int i = 1; i < 10; i++)
   {
   int angle = i * 36;
   double x = cos(angle * 3.15159 / 180);
   double y = sin(angle * 3.15159 / 180);
   pts->InsertPoint(i, y, x, radius_sphere_);
   }
   //pts->InsertPoint(0, 4, 0.403536, 0.652936);
   pts->Modified();
   ROS_INFO_STREAM("Points created");
   sphere->SetPoints(pts);
   //sphere->SetPolys()l
   ROS_INFO_STREAM("Points set");

  /*vtkSmartPointer<vtkPlatonicSolidSource> ico = vtkSmartPointer<vtkPlatonicSolidSource>::New();
  ico->SetSolidTypeToIcosahedron();
  ico->Update();
  //tesselate cells from icosahedron
  vtkSmartPointer<vtkLoopSubdivisionFilter> subdivide = vtkSmartPointer<vtkLoopSubdivisionFilter>::New();
  subdivide->SetNumberOfSubdivisions(1.0);
  subdivide->SetInputConnection(ico->GetOutputPort());
  ROS_INFO_STREAM("Points created");
  // Get camera positions
  vtkPolyData *sphere = subdivide->GetOutput();
  sphere->Update();*/

  std::vector<Eigen::Vector3f> cam_positions;
  cam_positions.resize(sphere->GetNumberOfPoints());
  for (int i = 0; i < sphere->GetNumberOfPoints(); i++)
  //for (int i=0; i<1; i++)
  {
    double cam_pos[3];
    sphere->GetPoint(i, cam_pos);
    cam_positions[i] = Eigen::Vector3f(float(cam_pos[0]), float(cam_pos[1]), float(cam_pos[2]));
    ROS_INFO_STREAM("sphere->getpoint(i, cam_pos), where cam_pos= "<< cam_pos[0]<<", "<<cam_pos[1]<<", "<<cam_pos[2]);
  }
  ROS_INFO_STREAM("Camera positions set");
  double camera_radius = radius_sphere_;
  double cam_pos[3];
  double first_cam_pos[3];

  first_cam_pos[0] = cam_positions[0][0] * radius_sphere_;
  first_cam_pos[1] = cam_positions[0][1] * radius_sphere_;
  first_cam_pos[2] = cam_positions[0][2] * radius_sphere_;
  //create renderer and window
  vtkSmartPointer<vtkRenderWindow> render_win = vtkSmartPointer<vtkRenderWindow>::New();
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  render_win->AddRenderer(renderer);
  render_win->SetSize(resolution_, resolution_);
  renderer->SetBackground(1.0, 0, 0);

  //create picker
  vtkSmartPointer<vtkWorldPointPicker> worldPicker = vtkSmartPointer<vtkWorldPointPicker>::New();

  vtkSmartPointer<vtkCamera> cam = vtkSmartPointer<vtkCamera>::New();
  cam->SetFocalPoint(0, 0, 0);

  Eigen::Vector3f cam_pos_3f = cam_positions[0];
  Eigen::Vector3f perp = cam_pos_3f.cross(Eigen::Vector3f::UnitY());
  cam->SetViewUp(perp[0], perp[1], perp[2]);

  cam->SetPosition(first_cam_pos);
  cam->SetViewAngle(view_angle_);
  cam->Modified();

  std::stringstream fileName, csvfilename;
  ROS_INFO_STREAM("Starting for loop");

  for (size_t i = 0; i < cam_positions.size(); i++)
  {
    cam_pos[0] = cam_positions[i][0];
    cam_pos[1] = cam_positions[i][1];
    cam_pos[2] = cam_positions[i][2];

    //create temporal virtual camera
    vtkSmartPointer<vtkCamera> cam_tmp = vtkSmartPointer<vtkCamera>::New();
    cam_tmp->SetViewAngle(view_angle_);

    Eigen::Vector3f cam_pos_3f(static_cast<float>(cam_pos[0]), static_cast<float>(cam_pos[1]),
                               static_cast<float>(cam_pos[2]));
    cam_pos_3f = cam_pos_3f.normalized();
    Eigen::Vector3f test = Eigen::Vector3f::UnitY();

    //If the view up is parallel to ray cam_pos - focalPoint then the transformation
    //is singular and no points are rendered...
    //make sure it is perpendicular
    if (fabs(cam_pos_3f.dot(test)) == 1)
    {
      //parallel, create
      test = cam_pos_3f.cross(Eigen::Vector3f::UnitX());
    }

    cam_tmp->SetViewUp(test[0], test[1], test[2]);

    for (int k = 0; k < 3; k++)
    {
      cam_pos[k] = cam_pos[k] * camera_radius;
    }

    cam_tmp->SetPosition(cam_pos);
    cam_tmp->SetFocalPoint(0, 0, 0);
    cam_tmp->Modified();

    //rotate model so it looks the same as if we would look from the new position
    vtkSmartPointer<vtkMatrix4x4> view_trans_inverted = vtkSmartPointer<vtkMatrix4x4>::New();
    vtkMatrix4x4::Invert(cam->GetViewTransformMatrix(), view_trans_inverted);
    vtkSmartPointer<vtkTransform> trans_rot_pose = vtkSmartPointer<vtkTransform>::New();
    trans_rot_pose->Identity();
    trans_rot_pose->Concatenate(view_trans_inverted);
    trans_rot_pose->Concatenate(cam_tmp->GetViewTransformMatrix());
    vtkSmartPointer<vtkTransformFilter> trans_rot_pose_filter = vtkSmartPointer<vtkTransformFilter>::New();
    trans_rot_pose_filter->SetTransform(trans_rot_pose);
    trans_rot_pose_filter->SetInputConnection(trans_filter_scale->GetOutputPort());

    //translate model so we can place camera at (0,0,0)
    vtkSmartPointer<vtkTransform> translation = vtkSmartPointer<vtkTransform>::New();
    translation->Translate(first_cam_pos[0] * -1, first_cam_pos[1] * -1, first_cam_pos[2] * -1);
    vtkSmartPointer<vtkTransformFilter> translation_filter = vtkSmartPointer<vtkTransformFilter>::New();
    translation_filter->SetTransform(translation);
    translation_filter->SetInputConnection(trans_rot_pose_filter->GetOutputPort());

    //modify camera
    cam_tmp->SetPosition(0, 0, 0);
    cam_tmp->SetFocalPoint(first_cam_pos[0] * -1, first_cam_pos[1] * -1, first_cam_pos[2] * -1);
    cam_tmp->Modified();

    //notice transformations for final pose
    vtkSmartPointer<vtkMatrix4x4> matrixRotModel = trans_rot_pose->GetMatrix();
    vtkSmartPointer<vtkMatrix4x4> matrixTranslation = translation->GetMatrix();

    mapper->SetInputConnection(translation_filter->GetOutputPort());
    mapper->Update();

    //render view
    vtkSmartPointer<vtkActor> actor_view = vtkSmartPointer<vtkActor>::New();
    actor_view->SetMapper(mapper);
    renderer->SetActiveCamera(cam_tmp);
    renderer->AddActor(actor_view);
    renderer->Modified();
    //renderer->ResetCameraClippingRange ();
    render_win->Render();

    //back to real scale transform
    vtkSmartPointer<vtkTransform> backToRealScale = vtkSmartPointer<vtkTransform>::New();
    backToRealScale->PostMultiply();
    backToRealScale->Identity();
    backToRealScale->Concatenate(matrixScale);
    backToRealScale->Concatenate(matrixTranslation);
    backToRealScale->Inverse();
    backToRealScale->Modified();
    backToRealScale->Concatenate(matrixTranslation);
    backToRealScale->Modified();

    Eigen::Matrix4f backToRealScale_eigen;
    backToRealScale_eigen.setIdentity();

    for (int x = 0; x < 4; x++)
      for (int y = 0; y < 4; y++)
        backToRealScale_eigen(x, y) = float(backToRealScale->GetMatrix()->GetElement(x, y));

    pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud(new pcl17::PointCloud<pcl17::PointXYZ>);

    cloud->points.resize(resolution_ * resolution_);
    cloud->width = resolution_ * resolution_;
    cloud->height = 1;

    double coords[3];
    float * depth = new float[resolution_ * resolution_];
    render_win->GetZbufferData(0, 0, resolution_ - 1, resolution_ - 1, &(depth[0]));

    int count_valid_depth_pixels = 0;
    for (int x = 0; x < resolution_; x++)
    {
      for (int y = 0; y < resolution_; y++)
      {
        float value = depth[y * resolution_ + x];
        if (value == 1.0)
          continue;

        worldPicker->Pick(x, y, value, renderer);
        worldPicker->GetPickPosition(coords);

        cloud->points[count_valid_depth_pixels].x = static_cast<float>(coords[0]);
        cloud->points[count_valid_depth_pixels].y = static_cast<float>(coords[1]);
        cloud->points[count_valid_depth_pixels].z = static_cast<float>(coords[2]);
        cloud->points[count_valid_depth_pixels].getVector4fMap() = backToRealScale_eigen
            * cloud->points[count_valid_depth_pixels].getVector4fMap();
        count_valid_depth_pixels++;
      }
    }

    delete[] depth;

    cloud->points.resize(count_valid_depth_pixels);
    cloud->width = count_valid_depth_pixels;

    //transform cloud to give camera coordinates instead of world coordinates!
    vtkSmartPointer<vtkMatrix4x4> view_transform = cam_tmp->GetViewTransformMatrix();
    Eigen::Matrix4f trans_view;
    trans_view.setIdentity();

    for (int x = 0; x < 4; x++)
      for (int y = 0; y < 4; y++)
        trans_view(x, y) = float(view_transform->GetElement(x, y));

    //NOTE: vtk view coordinate system is different than the standard camera coordinates (z forward, y down, x right)
    //thus, the fliping in y and z
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
      cloud->points[i].getVector4fMap() = trans_view * cloud->points[i].getVector4fMap();
      cloud->points[i].y *= -1.0f;
      cloud->points[i].z *= -1.0f;
    }

    ROS_INFO_STREAM("Cloud data constructed on "<<i<<" iteration, with "<<cloud->points.size()<<" points.");
    renderer->RemoveActor(actor_view);
    fileName << "pump_" << i << ".pcd";
    pcl17::io::savePCDFileASCII(fileName.str(), *cloud);
    fileName.clear();
    fileName.str("");
    //generated_views_.push_back (cloud);

    //create pose, from OBJECT coordinates to CAMERA coordinates!
    vtkSmartPointer<vtkTransform> transOCtoCC = vtkSmartPointer<vtkTransform>::New();
    transOCtoCC->PostMultiply();
    transOCtoCC->Identity();
    transOCtoCC->Concatenate(matrixCenter);
    transOCtoCC->Concatenate(matrixRotModel);
    transOCtoCC->Concatenate(matrixTranslation);
    transOCtoCC->Concatenate(cam_tmp->GetViewTransformMatrix());

    //NOTE: vtk view coordinate system is different than the standard camera coordinates (z forward, y down, x right)
    //thus, the fliping in y and z
    vtkSmartPointer<vtkMatrix4x4> cameraSTD = vtkSmartPointer<vtkMatrix4x4>::New();
    cameraSTD->Identity();
    cameraSTD->SetElement(0, 0, 1);
    cameraSTD->SetElement(1, 1, -1);
    cameraSTD->SetElement(2, 2, -1);

    transOCtoCC->Concatenate(cameraSTD);
    transOCtoCC->Modified();

    Eigen::Matrix4f pose_view;
    pose_view.setIdentity();

    for (int x = 0; x < 4; x++)
      for (int y = 0; y < 4; y++)
        pose_view(x, y) = float(transOCtoCC->GetMatrix()->GetElement(x, y));
    csvfilename << "pump_" << i << ".csv";

    std::ofstream posefile;
    posefile.open(csvfilename.str().c_str());

    Eigen::Vector3f eigen_vect = pose_view.block<3, 1>(0, 3);
    Eigen::Matrix3f model_rot = pose_view.block<3, 3>(0, 0);
    tf::Quaternion tf_quater;
    tf::Matrix3x3 tf_mod_matrix;
    tf_mod_matrix.setValue(model_rot(0, 0), model_rot(0, 1), model_rot(0, 2), model_rot(1, 0), model_rot(1, 1),
                           model_rot(1, 2), model_rot(2, 0), model_rot(2, 1), model_rot(2, 2));
    tf_mod_matrix.getRotation(tf_quater);
    tf::Vector3 tf_transl(eigen_vect(0), eigen_vect(1), eigen_vect(2));
    tf::Matrix3x3 rot_mat;
    rot_mat.setRotation(tf_quater);
    double roll, pitch, yaw;
    rot_mat.getRPY(roll, pitch, yaw);
    posefile << roll << ',';
    posefile << pitch << ',';
    posefile << yaw << ',';
    posefile << tf_transl.x() << ',';
    posefile << tf_transl.y() << ',';
    posefile << tf_transl.z() << ',';
    posefile<<endl;
    posefile.close();
    csvfilename.clear();
    csvfilename.str("");
  }

  /*  for (int i = 1; i < 10; i++)
   {
   int angle = i * 36;
   up_x_ = cos(angle * 3.15159 / 180);
   up_y_ = sin(angle * 3.15159 / 180);
   vis.setCameraPosition(pos_x_, pos_y_, pos_z_, view_x_, view_y_, view_z_, up_x_, up_y_, up_z_, 0);
   //vis.updateCamera();
   vis.setRepresentationToSurfaceForAllActors();
   //vis.spin();
   ROS_INFO_STREAM("Post vis spin");
   //call render in the visualizer to obtain a point cloud of the scene
   pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_out(new pcl17::PointCloud<pcl17::PointXYZ>());
   pcl17::PointCloud<pcl17::PointXYZ> cloud_filtered; //(new pcl17::PointCloud<pcl17::PointXYZ> ());
   std::vector<int> indices;
   vis.renderView(512, 512, cloud_out);

   //ROS_INFO_STREAM("Cloud in data is dense: "<<cloud_out->is_dense);
   cloud_out->is_dense = 0;
   //ROS_INFO_STREAM("Cloud in data is dense: "<<cloud_out->is_dense);
   pcl17::removeNaNFromPointCloud(*cloud_out, cloud_filtered, indices);

   fileName << "pump_" << angle << ".pcd";
   pcl17::io::savePCDFileASCII(fileName.str(), cloud_filtered);
   fileName.clear();
   fileName.str("");

   Eigen::Vector4f ei_v;
   Eigen::Quaternionf ei_quat;
   Eigen::Matrix4f ei_mat;
   Eigen::Affine3f ei_aff = vis.getViewerPose(0);
   Eigen::Affine3d eig_aff;
   eig_aff = ei_aff.cast<double>(); // Now this is OK</double></float>,3,3></double>,3,3></float>,3,3></double>,3,3>
   //Eigen::Affine3d eig_aff;
   //eig_aff.matrix()=ei_aff.matrix();
   tf::Transform part_pose;
   tf::TransformEigenToTF(eig_aff, part_pose);

   //tf::Transform part_pose;
   //part_pose.setRotation(tf_quater);
   //part_pose.setOrigin(tf_transl);
   tf::Quaternion part_q;
   part_q = part_pose.getRotation();
   tf::Matrix3x3 part_matrix;
   part_matrix.setRotation(part_q);
   tf::Matrix3x3 part_inv_mat;
   tf::Matrix3x3DoubleData part_inv_dd;
   part_matrix.serialize(part_inv_dd);
   //part_inv_mat.setValue(part_inv_dd.m_el[0].m_floats[0], part_inv_dd.m_el[1].m_floats[0], part_inv_dd.m_el[2].m_floats[0],
   //                    part_inv_dd.m_el[0].m_floats[1], part_inv_dd.m_el[1].m_floats[1], part_inv_dd.m_el[2].m_floats[1],
   //                  part_inv_dd.m_el[0].m_floats[2], part_inv_dd.m_el[1].m_floats[2], part_inv_dd.m_el[2].m_floats[2]);
   part_inv_mat.setValue(part_inv_dd.m_el[0].m_floats[0], part_inv_dd.m_el[0].m_floats[1],
   part_inv_dd.m_el[0].m_floats[2], part_inv_dd.m_el[1].m_floats[0],
   part_inv_dd.m_el[1].m_floats[1], part_inv_dd.m_el[1].m_floats[2],
   part_inv_dd.m_el[2].m_floats[0], part_inv_dd.m_el[2].m_floats[1],
   part_inv_dd.m_el[2].m_floats[2]);
   double roll, pitch, yaw;
   part_inv_mat.getRPY(roll, pitch, yaw);
   part_q.setRPY(roll, pitch, yaw);
   part_pose.setRotation(part_q);

   csvfilename << "pump_" << angle << ".csv";
   std::ofstream posefile;
   posefile.open(csvfilename.str().c_str());

   if (posefile.is_open())
   {
   //posefile << "TF(0 0), TF(0 1), TF(0 2), TF(0 3), TF(1 0), TF(1 1), "
   //  "TF(1 2), TF(1 3), TF(2 0),  TF(2 1), TF(2 2), TF(2 3) , "
   //"TF(3 0),  TF(3 1), TF(3 2), TF(3 3)\n";
   ROS_INFO_STREAM("posefile opened sucessfully");
   }
   else
   {
   ROS_INFO_STREAM("Unable to open STATFILE file");
   //return false;
   }
   std::vector<double> poses;
   tf::TransformDoubleData tf_dd;
   part_pose.serialize(tf_dd);
   ROS_INFO_STREAM("Part-pose "<<tf_dd.m_basis.m_el[1].m_floats[1]);
   posefile << roll << ',';
   posefile << pitch << ',';
   posefile << yaw << ',';
   posefile << part_pose.getOrigin().x() << ',';
   posefile << part_pose.getOrigin().y() << ',';
   posefile << part_pose.getOrigin().z() << ',';
   /*posefile << tf_dd.m_basis.m_el[0].m_floats[0] << ',';
   posefile << tf_dd.m_basis.m_el[0].m_floats[1] << ',';
   posefile << tf_dd.m_basis.m_el[0].m_floats[2] << ',';
   posefile << tf_dd.m_basis.m_el[0].m_floats[3] << ',';

   posefile << tf_dd.m_basis.m_el[1].m_floats[0] << ',';
   posefile << tf_dd.m_basis.m_el[1].m_floats[1] << ',';
   posefile << tf_dd.m_basis.m_el[1].m_floats[2] << ',';
   posefile << tf_dd.m_basis.m_el[1].m_floats[3] << ',';

   posefile << tf_dd.m_basis.m_el[2].m_floats[0] << ',';
   posefile << tf_dd.m_basis.m_el[2].m_floats[1] << ',';
   posefile << tf_dd.m_basis.m_el[2].m_floats[2] << ',';
   posefile << tf_dd.m_basis.m_el[2].m_floats[3] << ',';

   posefile << tf_dd.m_basis.m_el[3].m_floats[0] << ',';
   posefile << tf_dd.m_basis.m_el[3].m_floats[1] << ',';
   posefile << tf_dd.m_basis.m_el[3].m_floats[2] << ',';
   posefile << tf_dd.m_basis.m_el[3].m_floats[3] << ',';

   posefile.close();
   csvfilename.clear();
   csvfilename.str("");

   ROS_INFO_STREAM("Finished with "<<i<<"th loop");

   }*/
  /*
   vis.setCameraPosition(pos_x_,pos_y_,pos_z_,view_x_,view_y_,view_z_,up_x_,up_y_,up_z_,0);



   vis.camera_.window_size[0] = 480;
   vis.camera_.window_size[1] = 480;
   vis.camera_.pos[0] = 0;
   vis.camera_.pos[1] = 5;
   vis.camera_.pos[2] = 5;
   vis.camera_.focal[0] = 0;
   vis.camera_.focal[1] = 0;
   vis.camera_.focal[2] = 0;
   vis.camera_.view[0] = 0;
   vis.camera_.view[1] = 1;
   vis.camera_.view[2] = 0;

   vis.updateCamera();
   //vis.resetCamera();

   vis.setRepresentationToSurfaceForAllActors();
   vis.spin();

   //call render in the visualizer to obtain a point cloud of the scene
   pcl17::PointCloud<pcl17::PointXYZ>::Ptr cloud_out (new pcl17::PointCloud<pcl17::PointXYZ> ());
   vis.renderView(256,256, cloud_out);

   pcl17::io::savePCDFileASCII("scene.pcd",*cloud_out);*/

  return 0;
}

