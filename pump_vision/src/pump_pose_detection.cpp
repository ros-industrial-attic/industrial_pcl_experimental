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
//#include <cv_bridge/CvBridge.h>
#include <cv_bridge/cv_bridge.h>

#include <industrial_object_recognition_pcl17/object_recognition.h>

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

	ros::ServiceServer pump_Angle_Service_;

public:

	pump_image_processing(): it_(nh_)
	{
		image_pub_ = it_.advertise("Processed_Pump_Image", 1);
		//image_sub_ = it_.subscribe("prosilica/image_color", 1, &pump_image_processing::processPumpImage, this);
	}

	~pump_image_processing()
	{
	}

	Mat input_Img;  //Initial Image
	Mat binary_Img;	//Binary Image of Initial
	Mat binary_Removed_Img;  //Image the most processing is conducted on (Morph Removed)

	Mat largeCircle_Located;  //Convolved Image (Removed with Large Circle Filter)
	Mat smallCircle_Located;  //Convolved Image (Removed with Small Circle Filter)

	Mat large_Circle_Filter;  //Created large circle kernel
	Mat small_Circle_Filter;  //Created small circle kernel

	Mat diltation_Element;

	int threshold_Value;

	int circle_Width_Large;
	int circle_Width_Small;

	int largeRad;
	int smallRad;

	int outer_Circle_To_Center_Dist;

	Point Pump_Intersection_Small;
	Point Pump_Intersection_Large;

	float center_Calc_Dist;
	double center_Max_Dist;

	vector<Point> pump_Small_Hole_Points;
	vector<Point> extracted_Pump_MidPoints;
	vector<Point> pump_LongAxis_Points;

	float calculatedAngle;  //Calculated angle from the dot product between the horizontal and located port hole.
	float point_Line_Distance;  //Calculated distance between pump centerline and closest pump port.
	double max_point_Line_Distance; //Max distance allowed between pump centerline and closest pump port.

	String errorCode;


	void create_Dilation_Element(int sz)
	{
		try{
			diltation_Element = getStructuringElement(MORPH_RECT, Size(sz,sz));
		}catch(int a){
			errorCode = "Error";
			ROS_ERROR("Cannot create dilation structuring element.");
		}
	}

	Mat dialte_Image(Mat img, Mat structElem)
	{
		try{
			dilate(img, img, structElem);
		} catch(int a){
			errorCode = "Error";
			ROS_ERROR("Cannot dilate image.");
		}
		return img;
	}

	void create_Circular_Filters()
	{
		try{
			large_Circle_Filter = createHoughCircles(largeRad, circle_Width_Large);
			small_Circle_Filter = createHoughCircles(smallRad, circle_Width_Small);
		}catch(int a){
			errorCode = "Error";
			ROS_ERROR("Cannot create Hough Circle images.");
		}
	}

	//! Creates a binary image using the input value.
	void create_binary_image(double threshVal)
	{
		try{
			threshold(input_Img, binary_Img, threshVal, 255.0, CV_THRESH_BINARY);
		} catch(int a){
			errorCode = "Error";
			ROS_ERROR("Cannot threshold image.");
			return;
		}

		//imshow(WINDOW1, binary_Img);
		//waitKey(3);

	}

	Mat create_binary_image_Return(Mat img, double threshVal)
	{
		Mat bin_Img;
		try{
			threshold(img, bin_Img, threshVal, 255.0, CV_THRESH_BINARY);
			return bin_Img;
		} catch(int a){
			errorCode = "Error";
			ROS_ERROR("Cannot threshold image.");
			return bin_Img;
		}
	}

	Mat shrink_Binary_Image(Mat img, bool leaveSingleBlob)
	{
		Mat imgSweep = Mat::zeros(img.rows,img.cols, CV_8UC1);

		try{
			int s = img.step;
			int c = img.channels();

			vector<vector<Point> > contours;

			findContours( img, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

			vector<Moments> mu(contours.size() );
			vector<double> areas(contours.size());

			for( int i = 0; i < contours.size(); i++ ){
				mu[i] = moments( contours[i], false );
				areas[i] = contourArea(contours[i]);
			}

			double max_Area = 0.0;
			int iteration_Max_Size = 0;
			///  Get the mass centers:
			vector<Point2f> mc( contours.size() );
			for( int i = 0; i < contours.size(); i++ ){
				mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
				if (areas[i] > max_Area){
					max_Area = areas[i];
					iteration_Max_Size = i;
				}
			}
			if (!leaveSingleBlob){
				for (int ii = 0; ii < contours.size(); ii++){
					int x = (int)mc[ii].x;
					int y = (int)mc[ii].y;
					imgSweep.data[y*s+x*c+0] = 255;
				}
			}else{
				int x = (int)mc[iteration_Max_Size].x;
				int y = (int)mc[iteration_Max_Size].y;
				imgSweep.data[y*s+x*c+0] = 255;
			}

		}catch(int a){
			errorCode = "Error";
			ROS_ERROR("Cannot shrink binary image.");
		}
	    return imgSweep;
	}

	Mat convolveImage(Mat image, Mat convKernal)
	{
		Mat locatedCircles = Mat::zeros(image.rows,image.cols, CV_16UC1);;

		Mat located8Bit = Mat::zeros(image.rows,image.cols, CV_8UC1);;

		Mat imgInput_F = Mat::zeros(image.rows,image.cols, CV_16UC1);
		Mat kernelInput = Mat::zeros(convKernal.rows,convKernal.cols, CV_16UC1);

		image.convertTo(imgInput_F, CV_16UC1, 1.0/255.0);
		convKernal.convertTo(kernelInput, CV_16UC1, 1.0/255.0);

		Mat threshCheck;

		try{
			//ROS_ERROR("Here!1");
			filter2D(imgInput_F, locatedCircles, -1, kernelInput, Point(-1,-1), 0.0, BORDER_CONSTANT);
			//ROS_ERROR("Here!2");

			double max;
			int maxIdx[3];
			minMaxIdx(locatedCircles, 0, &max, 0, maxIdx);

			double threshVal = round(max * .9);
			int s_16 = locatedCircles.step;
			//int s_8 = located8Bit.step;
			int c = image.channels();


			//ROS_ERROR("%d, %d",locatedCircles.rows, locatedCircles.cols);
			//imshow(WINDOW3, locatedCircles);
			//waitKey(5000);

			Mat locatedFloat = Mat::zeros(image.rows,image.cols, CV_32FC1);



			locatedCircles.convertTo(locatedFloat, CV_32FC1, 1.0/max);

			threshold(locatedFloat, threshCheck, 0.75, 1.0, CV_THRESH_BINARY);

			//imshow(WINDOW1, threshCheck);
			//waitKey(5000);

			threshCheck.convertTo(threshCheck, CV_8UC1, 255.0);

			//ROS_ERROR("%s", threshCheck.)
			//locatedFloat.convertTo(located8Bit, CV_8UC1, 255.0);

			/*
			for (int ii = 0; ii < locatedCircles.rows; ii++){
				for (int jj = 0; jj < locatedCircles.cols; jj++){
					if (locatedCircles.data[ii*s_16 + jj*c + 0] > (int)threshVal){
						locatedCircles.data[ii*s_16 + jj*c + 0] = 255.0;
					}else{
						locatedCircles.data[ii*s_16 + jj*c + 0] = 0.0;
					}
				}
			}
			*/
			//locatedCircles.convertTo(located8Bit, CV_8UC1);
			//imshow(WINDOW4, located8Bit);


		} catch(int a){
			errorCode = "Error";
			ROS_ERROR("Cannot convolve image.");
		}
		return threshCheck;
	}

	///Convolve each circular filter with the removed, binary pump image
	///Creates the large and small locating images, respectively.
	void convolveCircluarFilters_With_PumpImg()
	{
		try{
			largeCircle_Located = convolveImage(binary_Removed_Img, large_Circle_Filter);
			smallCircle_Located = convolveImage(binary_Removed_Img, small_Circle_Filter);
		}catch(int a){
			errorCode = "Error";
			ROS_ERROR("Cannot convolve circular images.");
		}

		//imshow(WINDOW1, largeCircle_Located);
		//waitKey(5000);
		//imshow(WINDOW1, smallCircle_Located);
		//waitKey(5000);
	}

	Mat createConstantImage(Mat img, int val)
	{
		Mat M = Mat::ones(img.rows, img.cols, CV_64FC1);
		try{
			M = M * val;
		}catch(int a){
			ROS_ERROR("Cannot create constant image.");
			errorCode = "Error";
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
			errorCode = "Error";
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
			errorCode = "Error";
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
			errorCode = "Error";
			ROS_ERROR("Cannot create removed image.");
			return imgSweep;
		}
		return imgSweep;
	}

	Mat createHoughCircles(int radius, int variation)
	{
		Mat filter;

		int Kernal_size = radius * 2 + 11;

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
	    }catch(int e){
	    	errorCode = "Error";
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
			errorCode = "Error";
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
			//ROS_ERROR("%d, %d", pt.x, pt.y);
		}catch(int a){
			errorCode = "Error";
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
			errorCode = "Error";
			ROS_ERROR("Could not remove small circle points.");
			return retained_Pts;
		}

		if (retained_Pts.size() == 0){
			errorCode = "Error";
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
		//imshow(WINDOW4,binary_Removed_Img);
		//waitKey(5000);
	}

	/*
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
	*/

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
		point_Line_Distance = dist_Min;

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

		if (angle < 0){
			angle = -1;
			return (float)angle;
		}
		if ((Pump_Zero.y - centerPt.y) > 0){
			angle = 360 - angle;
		}

		if (angle < 0 || angle > 360){
			angle = -1;
			return (float)angle;
		}

		ROS_ERROR("Calculated angle is : %f", (float)angle);
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

		extracted_Pump_MidPoints = box_MidPoints;
		pump_LongAxis_Points = locate_LongestSegment(extracted_Pump_MidPoints);

		return drawing;
	}

	void drawErrorImage(bool showImg)
	{
		Mat imageDraw = Mat::zeros( binary_Img.rows, binary_Img.cols, CV_8UC3);
		line(imageDraw, Point(0,0),Point(binary_Img.cols - 1, binary_Img.rows - 1), Scalar(0,0,255), 2);
		line(imageDraw, Point(binary_Img.cols - 1,0),Point(0, binary_Img.rows - 1), Scalar(0,0,255), 2);

		errorCode = "Error";

		if (showImg){
			publishImage(imageDraw);
			//imshow(WINDOW4,imageDraw);
			//waitKey(3);
		}
	}

	void drawImageDetails(Mat img, Point cp_1, Point cp_2, vector<Point> small_circle_Pts, float angle, vector<Point> boxMidPoints, vector<Point> longAxis, Point shortLoc, bool showImg)
	{
		Mat imageDraw = Mat::zeros( img.rows, img.cols, CV_8UC3);
		cvtColor(img, imageDraw, CV_GRAY2BGR);

		Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
		circle(imageDraw, cp_1, largeRad, Scalar(255,0,0), 2);

		circle(imageDraw, cp_1, 2, Scalar(255,255,0),2);
		circle(imageDraw, cp_2, 2, Scalar(255,0,255),2);

		//ROS_ERROR("Here1");
		for(int ii = 0; ii < small_circle_Pts.size(); ii++){
			circle(imageDraw, small_circle_Pts[ii], 2, Scalar(128,255,0),2);
			circle(imageDraw, small_circle_Pts[ii], smallRad, Scalar(255,0,0), 2);
		}
		//ROS_ERROR("Here2");
		for(int ii = 0; ii < boxMidPoints.size(); ii++){
			circle(imageDraw, boxMidPoints[ii], 2, Scalar(128,0,255),2);
		}

		//ROS_ERROR("Here3");
		if (longAxis.size() != 0){
			line(imageDraw, Point(longAxis.at(0).x,longAxis.at(0).y), Point(longAxis.at(1).x,longAxis.at(1).y), Scalar(0,255,0),2);
		}

		//ROS_ERROR("Here4");
		if (shortLoc.x != 0.0 && shortLoc.y != 0.0){
			circle(imageDraw, shortLoc, 5, Scalar(0,0,255), 3);
		}

		angle = round((double)angle);
		if (angle > 360){
			angle = 0;
		}
		stringstream ss (stringstream::in | stringstream::out);
		ss << angle;

		string angle_Str = ss.str();
		//ROS_ERROR("Here5");
		putText(imageDraw, angle_Str, Point(10, imageDraw.rows*.98), FONT_HERSHEY_SCRIPT_SIMPLEX, 1.0, Scalar(0,0,255), 2);

		if (showImg){
			//imshow(WINDOW1,imageDraw);
			//waitKey(5000);
			publishImage(imageDraw);
		}
	}

	void publishImage(Mat img)
	{
		//image_transport::Publisher pub = it_.advertise("Processed_Pump_Image", 1);
		sensor_msgs::ImagePtr msg;

		cv_bridge::CvImage out_msg;
		//out_msg.header   = img->header;
		out_msg.encoding = "bgr8";
		out_msg.image    = img;

		//imshow(WINDOW1, img);
		//waitKey(3000);

		//ros::Rate loop_rate(5);
		//while (nh_.ok()) {
			image_pub_.publish(out_msg.toImageMsg());
		/*	ros::spinOnce();
			loop_rate.sleep();
		}
		*/
	}

	bool processPumpImage(const sensor_msgs::ImageConstPtr& msg)
	{
		errorCode = "Detected";

		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg,"bgr8");

		Mat img = cv_ptr->image;
		//ROS_ERROR("%d, %d", img.rows, img.cols);

		input_Img = resize_Mono_Img(img);

		create_binary_image(threshold_Value);//225

		//imshow(WINDOW1, binary_Img);
		//waitKey(3);
		//return;

		initial_morph_Process_Image();

		convolveCircluarFilters_With_PumpImg();

		smallCircle_Located = dialte_Image(smallCircle_Located, diltation_Element);
		smallCircle_Located = dialte_Image(smallCircle_Located, diltation_Element);

		smallCircle_Located = shrink_Binary_Image(smallCircle_Located, false);

		largeCircle_Located = dialte_Image(largeCircle_Located, diltation_Element);
		largeCircle_Located = dialte_Image(largeCircle_Located, diltation_Element);
		largeCircle_Located = shrink_Binary_Image(largeCircle_Located, true);

		pump_Small_Hole_Points = extract_Pump_Hole_Points(smallCircle_Located);

		bool objectDetected = false;
		if (pump_Small_Hole_Points.size() > 0){
			Pump_Intersection_Small = locatePumpIntersectionSmall(pump_Small_Hole_Points);
			objectDetected = true;
		} else{
			errorCode = "Error";
			ROS_ERROR("Could not locate small hole intersection!");
			return false;
		}

		if (objectDetected){
			pump_Small_Hole_Points = removeSmall_CirclePoints(pump_Small_Hole_Points, outer_Circle_To_Center_Dist, Pump_Intersection_Small, 5);
			if (pump_Small_Hole_Points.size() > 0){
				objectDetected = true;
			}else{
				objectDetected = false;
				errorCode = "Error";
				ROS_ERROR("Could not locate corresponding small holes!");
				return false;
			}
		}

		if (objectDetected){
			Pump_Intersection_Large = locatePumpIntersectionLarge(largeCircle_Located);
			center_Calc_Dist = calculateCenters_Distances(Pump_Intersection_Large, Pump_Intersection_Small);
		}

		if (objectDetected && (center_Calc_Dist > center_Max_Dist)){
			objectDetected = false;
			errorCode = "Error";
			ROS_ERROR("Center calculated distance (%f) is too far (%f)!", center_Calc_Dist, center_Max_Dist);
			return false;
		}

		if (objectDetected){
			pump_Orientation_Detection(binary_Img);
		}

		//return;
		Point zero_Side;
		if (objectDetected){
			zero_Side = findShortestPointDistance(pump_Small_Hole_Points, pump_LongAxis_Points);
		}

		if (objectDetected && (fabs(point_Line_Distance) < max_point_Line_Distance)){
			//ROS_ERROR("Point-line distance: %f", point_Line_Distance);
			objectDetected = true;
		}else{
			objectDetected = false;
			errorCode = "Error";
			ROS_ERROR("Line to point distance (%f) is too far (%f)!", point_Line_Distance, max_point_Line_Distance);
			return false;
		}

		//objectDetected = true;

		if (objectDetected){
			//ROS_ERROR("Here2");
			Point imgSide = Point(binary_Img.cols, Pump_Intersection_Small.y);
			//calculatedAngle = calc_VectorAngle(Pump_Intersection_Small, imgSide, zero_Side);
			calculatedAngle = calc_VectorAngle(Pump_Intersection_Large, imgSide, zero_Side);
			drawImageDetails(input_Img, Pump_Intersection_Large, Pump_Intersection_Small, pump_Small_Hole_Points, calculatedAngle, extracted_Pump_MidPoints, pump_LongAxis_Points, zero_Side, true);
			//ROS_ERROR("No Error!!!");
			return true;
		}else{
			drawErrorImage(true);
			//ROS_ERROR("Error!!!");
			return false;
		}
	}

	bool service_CB(industrial_object_recognition_pcl17::object_recognition::Request &main_request,
			industrial_object_recognition_pcl17::object_recognition::Response &main_response)
	{
		sensor_msgs::ImageConstPtr acquired_Img = ros::topic::waitForMessage<sensor_msgs::Image>("prosilica/image_color", nh_);

		if (processPumpImage(acquired_Img)){

		}else{
			calculatedAngle = -1;
			errorCode = "Error";
			//ROS_ERROR("Error!!!");
		}

		main_response.pose.rotation = calculatedAngle;
		main_response.label = errorCode;

		return true;
	}

	void initializePumpOrientationService()
	{
		ros::NodeHandle nh_private_("~");

		nh_private_.param("Pump_Large_Filter_Width", circle_Width_Large, int(3));
		nh_private_.param("Pump_Small_Filter_Width", circle_Width_Small, int(2));

		nh_private_.param("Pump_Large_Filter_Radius", largeRad, int(179));
		nh_private_.param("Pump_Small_Filter_Radius", smallRad, int(45));

		nh_private_.param("Max_Center_Dist", center_Max_Dist, double(20.0));
		nh_private_.param("Max_Point_To_Line_Dist", max_point_Line_Distance, double(10.0));
		nh_private_.param("Pump_Holes_To_Center_Dist", outer_Circle_To_Center_Dist, int(110));

		nh_private_.param("Binary_Threshold", threshold_Value, int(205));

		//ROS_ERROR("%d", threshold_Value);

		create_Circular_Filters();
		create_Dilation_Element(3);

		pump_Angle_Service_ = nh_.advertiseService("/pump_orientation_detection",&pump_image_processing::service_CB, this);
		//ros::spin();
	}

};




int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_reader");

  pump_image_processing pump_IP;

  pump_IP.initializePumpOrientationService();

  ros::spin();

  return 0;
}
