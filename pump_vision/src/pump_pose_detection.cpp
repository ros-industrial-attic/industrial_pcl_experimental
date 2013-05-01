/*
 * pump_pose_detection.cpp
 *
 *  Created on: Apr 22, 2013
 *      Author: twhitney
 */

#include <stdio.h>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>

//typedef boost::shared_ptr<CvImage> CvImagePtr;

static const char WINDOW1[] = "Image window 1";
static const char WINDOW2[] = "Image window 2";
static const char WINDOW3[] = "Image window 3";
static const char WINDOW4[] = "Image window 4";

namespace enc = sensor_msgs::image_encodings;

using namespace cv;
using namespace std;
RNG rng(12345);


class pump_image_processing
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

public:

	pump_image_processing(): it_(nh_)
	{
		//image_pub_ = it_.advertise("out", 1);
		image_sub_ = it_.subscribe("prosilica/image_color", 1, &pump_image_processing::processPumpImage, this);
	}

	~pump_image_processing()
	{
	}

	Mat input_Img;
	Mat binary_Img;
	Mat binary_Removed_Img;

	Mat largeCircle_Located;
	Mat smallCircle_Located;

	Mat large_Circle_Filter;
	Mat small_Circle_Filter;

	Mat flanges_Located;

	int threshhold_Value;

	int circle_Width_Large;
	int circle_Width_Small;

	int largeRad;
	int largeKernal_Size;

	int smallKernal_Size;
	int smallRad;

	int Flange_Center_Distance;

	int outer_Circle_To_Center_Dist;

	Mat diltation_Element;

	Point Pump_Intersection_Small;
	Point Pump_Intersection_Large;

	float center_Calc_Dist;
	float center_Max_Dist;

	vector<Point> pump_Small_Hole_Points;
	vector<Point> extracted_Pump_MidPoints;
	vector<Point> pump_LongAxis_Points;

	float calculatedAngle;



	void create_Dilation_Element(int sz)
	{
		try{
			diltation_Element = getStructuringElement(MORPH_RECT, Size(sz,sz));
		}catch(int a){
			ROS_ERROR("Cannot create dilation structuring element.");
		}
	}

	Mat dialte_Image(Mat img, Mat structElem)
	{
		try{
			dilate(img, img, structElem);
		} catch(int a){
			ROS_ERROR("Cannot dilate image.");
		}
		return img;
	}

	void create_Circular_Filters()
	{
		try{
			large_Circle_Filter = createHoughCircles(largeRad, largeKernal_Size, circle_Width_Large);
			small_Circle_Filter = createHoughCircles(smallRad, smallKernal_Size, circle_Width_Small);
		}catch(int a){
			ROS_ERROR("Cannot create Hough Circle images.");
		}
	}

	//! Creates a binary image using the input value.
	void create_binary_image(double threshVal)
	{
		try{
			threshold(input_Img, binary_Img, threshVal, 255.0, CV_THRESH_BINARY);
		} catch(int a){
			ROS_ERROR("Cannot threshold image.");
			return;
		}

	}

	Mat create_binary_image_Return(Mat img, double threshVal)
	{
		Mat bin_Img;
		try{
			threshold(img, bin_Img, threshVal, 255.0, CV_THRESH_BINARY);
			return bin_Img;
		} catch(int a){
			ROS_ERROR("Cannot threshold image.");
			return bin_Img;
		}
	}

	Mat shrink_Binary_Image(Mat img)
	{
		Mat imgSweep = Mat::zeros(img.rows,img.cols, CV_8UC1);

		try{
			int s = img.step;
			int c = img.channels();

			for (int ii = 0; ii < imgSweep.rows; ii++){
				for (int jj = 0; jj < imgSweep.cols; jj++){
					imgSweep.data[ii*s+jj*c+0] = img.data[ii*s+jj*c+0];
				}
			}

			bool stillProcessing = true;

			for (int ii = 0; ii < 500; ii++){
				if(stillProcessing){
					stillProcessing = false;
					for (int jj = 1; jj < img.rows -1; jj++){
						for (int kk = 1; kk < img.cols - 1; kk++){
							if (img.data[jj*s + kk*c + 0] == 255){
								int v1 = img.data[(jj-1)*s+(kk-1)*c+0];
								int v2 = img.data[(jj-1)*s+(kk)*c+0];
								int v3 = img.data[(jj-1)*s+(kk+1)*c+0];
								int v4 = img.data[(jj)*s+(kk-1)*c+0];
								int v5 = img.data[(jj)*s+(kk+1)*c+0];
								int v6 = img.data[(jj+1)*s+(kk-1)*c+0];
								int v7 = img.data[(jj+1)*s+(kk)*c+0];
								int v8 = img.data[(jj+1)*s+(kk+1)*c+0];
								int val = v1 + v2 + v3 + v4 + v5 + v6 + v7 + v8;
								if ((val > 0) && (val < 255 * 8)){
									//ROS_INFO("%d", val);
									stillProcessing = true;
									img.data[jj*s+kk*c+0] = 0;
								}
							}
						}
					}
				}else{
					ii = 499;
				}
			}
		}catch(int a){
			ROS_ERROR("Cannot shrink binary image.");
		}
	    return img;
	}

	Mat convolveImage(Mat image, Mat convKernal)
	{
		Mat locatedCircles;

		try{
		//ROS_INFO("HERE1");
			Mat imgConst = createConstantImage(image, 255);
			Mat circleConst = createConstantImage(convKernal, 255);

			Mat img = image.clone();
			Mat kernel = convKernal.clone();

			divide(image, imgConst, img, 1.0);

			divide(convKernal, circleConst, kernel, 1.0);

			filter2D(img, locatedCircles, 64, kernel, Point(-1,-1), 0.0, BORDER_CONSTANT);
		} catch(int a){
			ROS_ERROR("Cannot convolve image.");
		}
		return locatedCircles;
	}

	///Convolve each circular filter with the removed, binary pump image
	///Creates the large and small locating images, respectively.
	void convolveCircluarFilters_With_PumpImg()
	{
		try{
			largeCircle_Located = convolveImage(binary_Removed_Img, large_Circle_Filter);
			smallCircle_Located = convolveImage(binary_Removed_Img, small_Circle_Filter);
		}catch(int a){
			ROS_ERROR("Cannot convolve circular images.");
		}
	}

	Mat createConstantImage(Mat img, int val)
	{
		Mat M = Mat::ones(img.rows, img.cols, CV_8U);
		try{
			M = M * 255;
		}catch(int a){
			ROS_ERROR("Cannot create constant image.");
			return M;
		}
		return M;
	}


	void morph_Bridge()
	{
		Mat imgSweep;
		imgSweep = Mat::zeros(binary_Img.rows,binary_Img.cols, CV_8UC1);

		try{

			int s = binary_Img.step;
			int c = binary_Img.channels();

			for (int ii = 0; ii < imgSweep.rows; ii++){
				for (int jj = 0; jj < imgSweep.cols; jj++){
					imgSweep.data[ii*s+jj*c+0] = binary_Img.data[ii*s+jj*c+0];
				}
			}

			for (int ii = 1; ii < imgSweep.rows - 1; ii++){
				for (int jj = 1; jj < imgSweep.cols - 1; jj++){
					if (binary_Img.data[(ii)*s+(jj)*c+0] == 0){
						int v1 = binary_Img.data[(ii-1)*s+(jj-1)*c+0];
						int v2 = binary_Img.data[(ii-1)*s+(jj)*c+0];
						int v3 = binary_Img.data[(ii-1)*s+(jj+1)*c+0];
						int v4 = binary_Img.data[(ii)*s+(jj-1)*c+0];
						int v5 = binary_Img.data[(ii)*s+(jj+1)*c+0];
						int v6 = binary_Img.data[(ii+1)*s+(jj-1)*c+0];
						int v7 = binary_Img.data[(ii+1)*s+(jj)*c+0];
						int v8 = binary_Img.data[(ii+1)*s+(jj+1)*c+0];

						if ((v1  > 0) && ((v3 > 0) || (v5 > 0) || (v8 > 0) || (v7 > 0) || (v6 > 0))){
							imgSweep.data[(ii)*s+(jj)*c+0] = 255;
							continue;
						}
						if ((v2  > 0) && ((v6 > 0) || (v7 > 0) || (v8 > 0))){
							imgSweep.data[(ii)*s+(jj)*c+0] = 255;
							continue;
						}
						if ((v3  > 0) && ((v1 > 0) || (v4 > 0) || (v6 > 0) || (v7 > 0) || (v8 > 0))){
							imgSweep.data[(ii)*s+(jj)*c+0] = 255;
							continue;
						}
						if ((v4  > 0) && ((v3 > 0) || (v5 > 0) || (v8 > 0))){
							imgSweep.data[(ii)*s+(jj)*c+0] = 255;
							continue;
						}
						if ((v5  > 0) && ((v1 > 0) || (v4 > 0) || (v6 > 0))){
							imgSweep.data[(ii)*s+(jj)*c+0] = 255;
							continue;
						}
						if ((v6  > 0) && ((v1 > 0) || (v2 > 0) || (v3 > 0) || (v5 > 0) || (v8 > 0))){
							imgSweep.data[(ii)*s+(jj)*c+0] = 255;
							continue;
						}
						if ((v7  > 0) && ((v1 > 0) || (v2 > 0) || (v3 > 0))){
							imgSweep.data[(ii)*s+(jj)*c+0] = 255;
							continue;
						}
						if ((v8  > 0) && ((v6 > 0) || (v4 > 0) || (v1 > 0) || (v2 > 0) || (v3 > 0))){
							imgSweep.data[(ii)*s+(jj)*c+0] = 255;
							continue;
						}
					}
				}
			}
		}catch(int a){
			ROS_ERROR("Cannot create binary bridged image.");
			return;
		}
		binary_Img = imgSweep;
	}

	void morph_Majority()
	{
		Mat imgSweep;
		try{
		imgSweep = Mat::zeros(binary_Img.rows,binary_Img.cols, CV_8UC1);

		int s = binary_Img.step;
		int c = binary_Img.channels();

		for (int ii = 1; ii < imgSweep.rows - 1; ii++){
			for (int jj = 1; jj < imgSweep.cols - 1; jj++){
				if (binary_Img.data[(ii)*s+(jj)*c+0] == 0){
					int v1 = binary_Img.data[(ii-1)*s+(jj-1)*c+0];
					int v2 = binary_Img.data[(ii-1)*s+(jj)*c+0];
					int v3 = binary_Img.data[(ii-1)*s+(jj+1)*c+0];
					int v4 = binary_Img.data[(ii)*s+(jj-1)*c+0];
					int v5 = binary_Img.data[(ii)*s+(jj+1)*c+0];
					int v6 = binary_Img.data[(ii+1)*s+(jj-1)*c+0];
					int v7 = binary_Img.data[(ii+1)*s+(jj)*c+0];
					int v8 = binary_Img.data[(ii+1)*s+(jj+1)*c+0];
					int val = v1 + v2 + v3 + v4 + v5 + v6 + v7 + v8;
					if (val > 255 * 4)
					{
						imgSweep.data[(ii)*s+(jj)*c+0] = 255;
					}
				}
			}
		}
		//imshow(WINDOW1,imgSweep);
		//waitKey(0);

		add(imgSweep,binary_Img, binary_Img);
		}catch(int a){
			ROS_ERROR("Cannot create majority image.");
		}
		//imshow(WINDOW1,binary_Img);
		//waitKey(0);
		//binary_Img = imgSweep;
	}


	Mat morph_Removal(Mat img)
	{
		Mat imgSweep;
		try{
			imgSweep = Mat::zeros(img.rows,img.cols, CV_8UC1);
			int s = img.step;
			int c = img.channels();

			for (int ii = 0; ii < imgSweep.rows; ii++){
				for (int jj = 0; jj < imgSweep.cols; jj++){
					imgSweep.data[ii*s+jj*c+0] = img.data[ii*s+jj*c+0];
				}
			}

			for (int ii = 1; ii < imgSweep.rows - 1; ii++){
				for (int jj = 1; jj < imgSweep.cols - 1; jj++){
					if (img.data[(ii)*s+(jj)*c+0] == 255){
						int v2 = img.data[(ii-1)*s+(jj)*c+0];
						int v4 = img.data[(ii)*s+(jj-1)*c+0];
						int v5 = img.data[(ii)*s+(jj+1)*c+0];
						int v7 = img.data[(ii+1)*s+(jj)*c+0];
						if ((v2 == 255) && (v4 == 255) && (v5 == 255) && (v7 == 255)){
							imgSweep.data[ii*s+jj*c+0] = 0;
						}
					}
				}
			}
		}catch(int a){
			ROS_ERROR("Cannot create removed image.");
			return imgSweep;
		}
		return imgSweep;
	}

	Mat createHoughCircles(int radius, int Kernal_size, int variation)
	{
		Mat filter;
		filter = Mat::zeros(Kernal_size,Kernal_size, CV_8UC1);

	    int s = filter.step;
	    int c = filter.channels();

		for(int ii = 0; ii < Kernal_size; ii++){
			for(int jj = 0; jj < Kernal_size; jj++){
				float d = sqrt((ii - Kernal_size / 2)*(ii - Kernal_size / 2) + (jj - Kernal_size / 2)*(jj - Kernal_size / 2));
				if ((d > (float)(radius - variation)) && (d < (float)(radius + variation))){
					filter.data[ii*s+jj*c+0]= 255;
				}
			}
		}
		return filter;
	}

	Mat resize_Mono_Img(Mat inputImg)
	{
		Mat outputImg;
		Mat halfImg;
		vector<Mat> bgr_planes;

	    try
	    {
	    	Size sz = Size(round(inputImg.cols * 0.5), round(inputImg.rows * 0.5));
	    	resize(inputImg, halfImg, sz, 0.0f, 0.0f, CV_INTER_LINEAR);

	    	split( halfImg, bgr_planes);
	    }
	    catch(int e)
	    {
	      ROS_ERROR("Image Splitting Exception");
	      return halfImg;
	    }

		return bgr_planes[2];
	}

	int calculate_Small_Circle_Thresh(Mat img, double val)
	{
		double max;
		int maxIdx[3];
		minMaxIdx(img, 0, &max, 0, maxIdx);
		//double mx = max_element(img.begin<double>(),img.end<double>());
		int percentVal = (int)round(val * max);
		//ROS_INFO("%d", percentVal);
		//int percentVal = 1;
		return percentVal;
	}

	vector<Point> extract_Pump_Hole_Points(Mat img)
	{
		vector<Point> pts;

	    int s = img.step;
	    int c = img.channels();

		int count;

		for (int ii = 0; ii < img.rows; ii++){
			for (int jj = 0; jj < img.cols; jj++){
				if (img.data[ii*s+jj*c+0] == 255){
					count++;
					Point pt = Point(jj,ii);
					pts.push_back(pt);
				}
			}
		}
		if (count == 0){
			ROS_ERROR("No pump hole points detected.");
			//ROS_ERROR("Here!1");
			//throw 'No pump hole points detected.';
			return pts;
		}
		return pts;
	}

	Point locatePumpIntersectionSmall(vector<Point> pts)
	{
		Mat imgSweep = Mat::zeros( binary_Img.rows, binary_Img.cols, CV_8UC1);
		Point pt;
		try{
			int s = imgSweep.step;
			int c = imgSweep.channels();

			for (int ii = 0; ii < pts.size();  ii++){
				int x = pts.at(ii).x;
				int y = pts.at(ii).y;
				for (int jj = 0; jj < imgSweep.rows; jj++){
					for (int kk = 0; kk < imgSweep.cols; kk++){
						float dist = sqrt((double)(jj - y)*(jj - y) + (double)(kk - x)*(kk - x));
						if ((dist > outer_Circle_To_Center_Dist - 5) && (dist < outer_Circle_To_Center_Dist + 5)){
							imgSweep.data[jj*s+kk*c+0] = imgSweep.data[jj*s+kk*c+0] + 1;
						}
					}
				}
			}

			double max;
			int maxIdx[3];
			minMaxIdx(imgSweep, 0, &max, 0, maxIdx);
			double multiply_Val = 255 / max;
			Scalar val = (Scalar)multiply_Val;
			multiply(imgSweep,val, imgSweep);

			pt = Point(maxIdx[1], maxIdx[0]);
		}catch(int a){
			ROS_ERROR("Could not locate pump intersections (Small).");
			return pt;
		}

	    return pt;
	}

	Point locatePumpIntersectionLarge(Mat img)
	{
		double max;
		Point pt;
		try{
			int maxIdx[3];
			minMaxIdx(img, 0, &max, 0, maxIdx);
			pt = Point(maxIdx[1], maxIdx[0]);
		}catch(int a){
			ROS_ERROR("Could not locate pump intersections (Large).");
			return pt;
		}

		return pt;
	}

	float calculateCenters_Distances(Point p1, Point p2)
	{
		double xa = (double)(p1.x - p2.x);
		double ya = (double)(p1.y - p2.y);


		double dist = sqrt(xa*xa +ya*ya);
		//ROS_INFO("%f, %f, %f",xa, ya, dist);
		return (float)dist;
	}

	vector<Point> removeSmall_CirclePoints(vector<Point> pts, int radius, Point centerPt, int variation)
	{
		vector<Point> retained_Pts;

		try{
			for(int ii = 0; ii < pts.size(); ii++){
				double x = (double)((pts.at(ii).x) - centerPt.x);
				double y = (double)((pts.at(ii).y) - centerPt.y);
				double dist = sqrt(x*x + y*y);
				//ROS_INFO("%f, %f, %f",x, y, dist);
				if ((dist > (double)(radius - variation)) && (dist < (double)(radius + variation))){
					retained_Pts.push_back(Point((float)pts.at(ii).x,(float)pts.at(ii).y));
					//ROS_INFO("%f, %f",pts.at(ii).x, pts.at(ii).y);
				}
			}
		}catch(int a){
			ROS_ERROR("Could not remove small circle points.");
			return retained_Pts;
		}

		if (retained_Pts.size() == 0){
			ROS_ERROR("Could not remove small circle points.");
			//throw "Could not remove small circle points.";
		}

		return retained_Pts;
	}

	vector<Point> locate_LongestSegment(vector<Point> pts)
	{
		vector<float> lengths;
		vector<Point> longestPoints;

		float len = 0;

		for (int ii = 0; ii < pts.size() - 1; ii++){
			for (int jj = ii + 1; jj < pts.size(); jj++){
				float x = (float)(pts.at(ii).x - pts.at(jj).x);
				float y = (float)(pts.at(ii).y - pts.at(jj).y);
				float val = sqrt(x*x + y*y);

				if (val > len){
					vector<Point> vals;

					vals.push_back(Point((float)pts.at(ii).x, (float)pts.at(ii).y));
					vals.push_back(Point((float)pts.at(jj).x, (float)pts.at(jj).y));

					longestPoints.clear();
					longestPoints = vals;
					len = val;
				}
			}
		}

		for (int ii = 0; ii < longestPoints.size(); ii++){
			//ROS_INFO("%f, %f", (float)longestPoints.at(ii).x, (float)longestPoints.at(ii).y);
		}

		return longestPoints;
	}

	///Morphologically Bridge, Majority and Remove the input image
	///Updates the input binary image
	///Creates the removed image
	void initial_morph_Process_Image()
	{
		morph_Bridge();
		morph_Majority();
		binary_Removed_Img = morph_Removal(binary_Img);
	}

	Mat extractFlangeLocations(Mat img, Point pt)
	{
		Mat imgSweep = Mat::zeros( img.rows, img.cols, CV_8UC1);
	    int s = img.step;
	    int c = img.channels();

	    int x = pt.x;
	    int y = pt.y;

	    for(int ii = 0; ii < img.rows; ii++){
	    	for(int jj = 0; jj < img.cols; jj++){
	    		float dist =sqrt((double)((ii - y)*(ii - y) + (jj - x)*(jj-x)));
	    		if ((dist > (double)(Flange_Center_Distance -3)) && (dist < (double)(Flange_Center_Distance + 3))){
	    			imgSweep.data[ii*s+jj*c+0] = img.data[ii*s+jj*c+0];
	    		}
	    	}
	    }

	    //imshow(WINDOW2,imgSweep);
	    //waitKey(0);

	    return imgSweep;
	}

	Point findShortestPointDistance(vector<Point> pts, vector<Point> line)
	{
		Point pt;

		float x1 = line.at(0).x;
		float y1 = line.at(0).y;
		float x2 = line.at(1).x;
		float y2 = line.at(1).y;

		float dist_Min = 10000;

		for (int ii = 0; ii < pts.size(); ii++){
			float x0 = pts.at(ii).x;
			float y0 = pts.at(ii).y;
			float dist = (fabs((x2-x1)*(y1-y0)-(x1-x0)*(y2-y1)))/(sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1)));
			if (dist < dist_Min){
				dist_Min = dist;
				pt = Point(x0, y0);
			}
		}

		//ROS_INFO("%f, %f", pt.x, pt.y);
		return pt;
	}


	float calc_VectorAngle(Point centerPt, Point rightSide, Point Pump_Zero)
	{
		Point vect_Horiz = Point((double)rightSide.x - centerPt.x,(double)0);
		Point vect_Pump = Point(double(Pump_Zero.x - centerPt.x), (double)centerPt.y - Pump_Zero.y);

		double magH = sqrt((vect_Horiz.x * vect_Horiz.x ) + (vect_Horiz.y * vect_Horiz.y));
		double magP = sqrt((vect_Pump.x * vect_Pump.x ) + (vect_Pump.y * vect_Pump.y));

		double dotP = vect_Horiz.x * vect_Pump.x + vect_Horiz.y * vect_Pump.y;

		double angle = acos((dotP/ (magH * magP)))* 180 / 3.14;

		if ((Pump_Zero.y - centerPt.y) > 0){
			angle = 360 - angle;
		}
		//ROS_INFO("%f", angle);

		return (float)angle;
	}

	Mat pump_Orientation_Detection(Mat img)
	{
		Mat threshold_output;

		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;

		img = dialte_Image(img, diltation_Element);
		img = dialte_Image(img, diltation_Element);

		/// Find contours
		findContours( img, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

		/// Find the rotated rectangles and ellipses for each contour
		vector<RotatedRect> minRect( contours.size() );
		vector<RotatedRect> minEllipse( contours.size() );

		for( int i = 0; i < contours.size(); i++ ){
			minRect[i] = minAreaRect( Mat(contours[i]) );
			if( contours[i].size() > 5 ){
				minEllipse[i] = fitEllipse( Mat(contours[i]) );
			}
		}

		float area_Max = 0;
		float Rect_angle = 0;

		Point2f rect_points_max[4];
		/// Draw contours + rotated rects + ellipses
		Mat drawing = Mat::zeros( img.size(), CV_8UC3 );
		for( int i = 0; i< contours.size(); i++ ){
			Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );

			// contour
			drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
			// ellipse
			ellipse( drawing, minEllipse[i], color, 2, 8 );
			float area = minEllipse[i].size.area();
			if (area > area_Max){
				area_Max = area;
				Rect_angle = minEllipse[i].angle;
				minRect[i].points( rect_points_max );
			}

			// rotated rectangle
			Point2f rect_points[4]; minRect[i].points( rect_points );
			for( int j = 0; j < 4; j++ ){
				line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
			}
		}

		vector<Point> box_MidPoints;
		for(int ii = 0; ii < 4; ii++){
			//ROS_INFO("%f, %f, %f, %f",rect_points_max[ii].x,rect_points_max[ii].y,rect_points_max[(ii+1)%4].x,rect_points_max[(ii+1)%4].y);
			float dx = (rect_points_max[ii].x + rect_points_max[(ii+1)%4].x) / 2;
			float dy = (rect_points_max[ii].y + rect_points_max[(ii+1)%4].y) / 2;
			//ROS_INFO("%f, %f", dx, dy);
			box_MidPoints.push_back(Point(dx,dy));
		}

		//ROS_INFO("%f, %f", area_Max, Rect_angle);

		extracted_Pump_MidPoints = box_MidPoints;

		pump_LongAxis_Points = locate_LongestSegment(extracted_Pump_MidPoints);

		//imshow( WINDOW3, drawing );
		//waitKey(0);

	    return drawing;
	}

	void drawErrorImage()
	{
		Mat imageDraw = Mat::zeros( binary_Img.rows, binary_Img.cols, CV_8UC3);
		line(imageDraw, Point(0,0),Point(binary_Img.cols - 1, binary_Img.rows - 1), Scalar(0,0,255), 2);
		line(imageDraw, Point(binary_Img.cols - 1,0),Point(0, binary_Img.rows - 1), Scalar(0,0,255), 2);

	    imshow(WINDOW4,imageDraw);
	    waitKey(3);
	}

	void drawImageDetails(Mat img, Point cp_1, Point cp_2, vector<Point> small_circle_Pts, float angle, vector<Point> boxMidPoints, vector<Point> longAxis, Point shortLoc)
	{
		Mat imageDraw = Mat::zeros( img.rows, img.cols, CV_8UC3);
		cvtColor(img, imageDraw, CV_GRAY2BGR);

		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		circle(imageDraw, cp_1, largeRad, Scalar(255,0,0), 2);

		circle(imageDraw, cp_1, 2, Scalar(255,255,0),2);
		circle(imageDraw, cp_2, 2, Scalar(255,0,255),2);

		for(int ii = 0; ii < small_circle_Pts.size(); ii++){
			circle(imageDraw, small_circle_Pts[ii], 2, Scalar(128,255,0),2);
		}

		for(int ii = 0; ii < boxMidPoints.size(); ii++){
			circle(imageDraw, boxMidPoints[ii], 2, Scalar(128,0,255),2);
		}

		line(imageDraw, Point(longAxis.at(0).x,longAxis.at(0).y), Point(longAxis.at(1).x,longAxis.at(1).y), Scalar(0,255,0),2);

		circle(imageDraw, shortLoc, 5, Scalar(0,0,255), 3);

		angle = round((double)angle);
		if (angle > 360){
			angle = 0;
		}
		stringstream ss (stringstream::in | stringstream::out);
		ss << angle;

		string angle_Str = ss.str();

		putText(imageDraw, angle_Str, Point(10, imageDraw.rows*.98), FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, Scalar(0,0,255), 2);


	    imshow(WINDOW4,imageDraw);
	    waitKey(3);
	}

	void processPumpImage(const sensor_msgs::ImageConstPtr& msg)
	{
		//sensor_msgs::CvBridge bridge;
		//ROS_INFO("Here!!!!1");


		//try
		//{

		//sensor_msgs::CvBridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
		//cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");

		//imshow(WINDOW1, cv_ptr->image);
		//waitKey(3);


		Mat img = cv_ptr->image;
		//imshow(WINDOW4,img);
		//waitKey(3);

		input_Img = resize_Mono_Img(img);


		create_binary_image(235);
		initial_morph_Process_Image();

		convolveCircluarFilters_With_PumpImg();

        int threshVal = calculate_Small_Circle_Thresh(smallCircle_Located, 0.75);

		smallCircle_Located = create_binary_image_Return(smallCircle_Located, (double)threshVal);
		//imshow(WINDOW1, pump_IP.smallCircle_Located);
		//waitKey(0);

		smallCircle_Located = dialte_Image(smallCircle_Located, diltation_Element);
		//imshow(WINDOW1, pump_IP.smallCircle_Located);
		//waitKey(0);

		smallCircle_Located = shrink_Binary_Image(smallCircle_Located);

		pump_Small_Hole_Points = extract_Pump_Hole_Points(smallCircle_Located);


		bool objectDetected = false;
		if (pump_Small_Hole_Points.size() > 0){
			Pump_Intersection_Small = locatePumpIntersectionSmall(pump_Small_Hole_Points);
			objectDetected = true;
		} else{
			ROS_ERROR("Could not locate small hole intersection!");
		}

		if (objectDetected){
			pump_Small_Hole_Points = removeSmall_CirclePoints(pump_Small_Hole_Points, outer_Circle_To_Center_Dist, Pump_Intersection_Small, 5);
			if (pump_Small_Hole_Points.size() > 0){
				objectDetected = true;
			}else{
				objectDetected = false;
				ROS_ERROR("Could not locate corresponding small holes!");
			}
		}

		if (objectDetected){
			Pump_Intersection_Large = locatePumpIntersectionLarge(largeCircle_Located);
			center_Calc_Dist = calculateCenters_Distances(Pump_Intersection_Large, Pump_Intersection_Small);
		}

		if (center_Calc_Dist > center_Max_Dist){
			objectDetected = false;
			ROS_ERROR("Center calculated distance (%f) is too far (%f)!", center_Calc_Dist, center_Max_Dist);
		}

		if (objectDetected){
			pump_Orientation_Detection(binary_Img);
		}

		if (objectDetected){
			Point zero_Side = findShortestPointDistance(pump_Small_Hole_Points, pump_LongAxis_Points);

			Point imgSide = Point(binary_Img.cols, Pump_Intersection_Small.y);
			calculatedAngle = calc_VectorAngle(Pump_Intersection_Small, imgSide, zero_Side);
			drawImageDetails(input_Img, Pump_Intersection_Large, Pump_Intersection_Small, pump_Small_Hole_Points, calculatedAngle, extracted_Pump_MidPoints, pump_LongAxis_Points, zero_Side);
		}else{
			drawErrorImage();
		}
			//}
		//}
	}

};




int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_reader");

  pump_image_processing pump_IP;


  pump_IP.largeKernal_Size = 375; //391
  pump_IP.smallKernal_Size = 95;//105

  pump_IP.circle_Width_Large = 1;
  pump_IP.circle_Width_Small = 2;
  pump_IP.largeRad = 181;  //190
  pump_IP.smallRad = 44; //47

  pump_IP.center_Max_Dist = 20;

  pump_IP.Flange_Center_Distance = 214;
  pump_IP.outer_Circle_To_Center_Dist = 115; //115

  pump_IP.threshhold_Value = 235;

  pump_IP.create_Circular_Filters();
  pump_IP.create_Dilation_Element(3);


  //Mat image_input = imread("/home/twhitney/Desktop/2013_04_22_Pump_Housing_Vision/Test Images 3/RecordedImage_GE1350C (02-2031C)_00-0F-31-01-B9-9A_250.tif");
  //cvNamedWindow("view");
  //cvStartWindowThread();


  //ros::NodeHandle nh;

  //image_transport::ImageTransport it(nh);

  //image_transport::Subscriber sub = it.subscribe("/prosilica/image_raw", 1, processPumpImage);
  ros::spin();
  //cvDestroyWindow("view");
  /*
  //image_transport::Subscriber sub = it.subscribe("in_image_base_topic", 1, imageCallback);
  //image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);

  pump_IP.input_Img = pump_IP.resize_Mono_Img(image_input);

  //ROS_INFO("%d, %d", pump_IP.input_Img.rows, pump_IP.input_Img.cols);
  pump_IP.create_binary_image(235);

  //imshow(WINDOW1, pump_IP.binary_Img);
  //waitKey(0);

  pump_IP.initial_morph_Process_Image();


  pump_IP.convolveCircluarFilters_With_PumpImg();

  int threshVal = pump_IP.calculate_Small_Circle_Thresh(pump_IP.smallCircle_Located, 0.75);

  pump_IP.smallCircle_Located = pump_IP.create_binary_image_Return(pump_IP.smallCircle_Located, (double)threshVal);
  imshow(WINDOW1, pump_IP.smallCircle_Located);
  waitKey(0);

  pump_IP.smallCircle_Located = pump_IP.dialte_Image(pump_IP.smallCircle_Located, pump_IP.diltation_Element);
  imshow(WINDOW1, pump_IP.smallCircle_Located);
  waitKey(0);

  pump_IP.smallCircle_Located = pump_IP.shrink_Binary_Image(pump_IP.smallCircle_Located);

  pump_IP.pump_Small_Hole_Points = pump_IP.extract_Pump_Hole_Points(pump_IP.smallCircle_Located);
  pump_IP.Pump_Intersection_Small = pump_IP.locatePumpIntersectionSmall(pump_IP.pump_Small_Hole_Points);

  pump_IP.pump_Small_Hole_Points = pump_IP.removeSmall_CirclePoints(pump_IP.pump_Small_Hole_Points, pump_IP.outer_Circle_To_Center_Dist, pump_IP.Pump_Intersection_Small, 5);
  //ROS_INFO("%d, %d", pump_IP.Pump_Intersection_Small.x, pump_IP.Pump_Intersection_Small.y);

  pump_IP.Pump_Intersection_Large = pump_IP.locatePumpIntersectionLarge(pump_IP.largeCircle_Located);
  //ROS_INFO("%d, %d", pump_IP.Pump_Intersection_Large.x, pump_IP.Pump_Intersection_Large.y);

  pump_IP.center_Calc_Dist = pump_IP.calculateCenters_Distances(pump_IP.Pump_Intersection_Large, pump_IP.Pump_Intersection_Small);
  //ROS_INFO("%f",pump_IP.center_Calc_Dist);

  if (pump_IP.center_Calc_Dist > pump_IP.center_Max_Dist)
  {

  }

  imshow(WINDOW1, pump_IP.binary_Img);
  waitKey(0);

  //pump_IP.flanges_Located = pump_IP.extractFlangeLocations(pump_IP.binary_Img, pump_IP.Pump_Intersection_Small);


  pump_IP.convexHull_FlangeImage(pump_IP.binary_Img);

  Point zero_Side = pump_IP.findShortestPointDistance(pump_IP.pump_Small_Hole_Points, pump_IP.pump_LongAxis_Points);

  Point imgSide = Point(pump_IP.binary_Img.cols, pump_IP.Pump_Intersection_Small.y);
  pump_IP.calculatedAngle = pump_IP.calc_VectorAngle(pump_IP.Pump_Intersection_Small, imgSide, zero_Side);

  pump_IP.drawImageDetails(pump_IP.input_Img, pump_IP.Pump_Intersection_Large, pump_IP.Pump_Intersection_Small, pump_IP.pump_Small_Hole_Points, pump_IP.calculatedAngle, pump_IP.extracted_Pump_MidPoints, pump_IP.pump_LongAxis_Points, zero_Side);



  return 0;
  */
}
