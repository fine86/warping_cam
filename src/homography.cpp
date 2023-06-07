#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "fiducial_msgs/FiducialArray.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <image_transport/image_transport.h>
#include "cv_bridge/cv_bridge.h"
#include <iostream>
#include <vector>


class homographyTransform{
public:

  homographyTransform():nh(), it(nh)
  {
    sub = nh.subscribe("/fiducial_vertices", 1, &homographyTransform::homographyCallback, this);
    img_sub = it.subscribe("/usb_cam/image_raw", 1, &homographyTransform::transformCallback, this);
    img_pub = it.advertise("/homography", 1);
  }

  // Calculate extrinsic parameter
  void extrinsicCalibration(std::vector<cv::Point2f> imagePoints, cv::Mat intrinsic, cv::Mat distortion){
	  std::vector<cv::Point3f> objectPoints;

	  cv::Point3f a(0, 0, 0);
	  objectPoints.push_back(a);

	  a = cv::Point3f(800, 0, 0);
	  objectPoints.push_back(a);

	  a = cv::Point3f(800, 800, 0);
	  objectPoints.push_back(a);

	  a = cv::Point3f(0, 800, 0);
	  objectPoints.push_back(a);	

	  cv::Mat rvec, tvec;

	  cv::solvePnP(objectPoints, imagePoints, intrinsic, distortion, rvec, tvec);

	  // calculate R
	  cv::Rodrigues(rvec, R);
	  cv::Mat R_inv = R.inv();

	  cam_pos = -R_inv*tvec;

	  
	  double* p = (double *)cam_pos.data;
	  
	  pos_x = p[0];
	  pos_y = p[1];
	  pos_z = p[2];
	  

	  // Calculate yaw, pitch, roll
	  double unit_z[] = {0, 0, 1};
	  cv::Mat Zc(3, 1, CV_64FC1, unit_z);
	  cv::Mat Zw = R_inv*Zc;
	  double* zw = (double *)Zw.data;

	  yaw = std::atan2(zw[1], zw[0]) - 3.141592/2;

	  pitch = std::atan2(zw[2], std::sqrt(std::pow(zw[0], 2) + std::pow(zw[1], 2)));

	  double unit_x[] = {1, 0, 0};

	  cv::Mat Xc(3, 1, CV_64FC1, unit_x);
	  cv::Mat Xw = R_inv*Xc;

	  double* xw = (double *)Xw.data;
	  double yaw_x[] = {std::cos(yaw), std::sin(yaw), 0};

	  roll = std::acos(xw[0]*yaw_x[0] + xw[1]*yaw_x[1] + xw[2]*yaw_x[2]);

	  if(xw[2] < 0)
		  roll = -roll;



  }

  // function to calculate homography matrix
  void homographyCallback(const fiducial_msgs::FiducialArray::ConstPtr& msg){
    int len = msg->fiducials.size();

    if(len>=4){
	    int id, isFour=0;
	    std::vector<cv::Point2f> arucoEdge, perspectiveEdge;
	    cv::Point a1, a2, a3, a4, p1, p2, p3, p4;

    	    for(int i=0; i!=len; i++){
	    	id = msg->fiducials[i].fiducial_id;
	    	//Detect edge markers. It's possible to change id.
	    	switch(id){
			    case 2:
				    a1=cv::Point2f(msg->fiducials[i].x0, msg->fiducials[i].y0);
			      	    isFour++;
			    	    break;
		    	    case 3:
			    	    a2=cv::Point2f(msg->fiducials[i].x0, msg->fiducials[i].y0);
			    	    isFour++;
			    	    break;
		    	    case 1:
			    	    a3=cv::Point2f(msg->fiducials[i].x0, msg->fiducials[i].y0);
			    	    isFour++;
			    	    break;
		    	    case 0:
				    a4=cv::Point2f(msg->fiducials[i].x0, msg->fiducials[i].y0);
				    isFour++;
				    break;
		}
	    }

	    if(isFour==4){
  		    p1=cv::Point2f(0, 0);
		    p2=cv::Point2f(width, 0);
		    p3=cv::Point2f(width, height);
		    p4=cv::Point2f(0, height);

		    std::vector<cv::Point2f> arucoEdge={a1, a2, a3, a4};
		    std::vector<cv::Point2f> perspectiveEdge={p1, p2, p3, p4};

		    homoMat = cv::findHomography(arucoEdge, perspectiveEdge);

		    std::vector<double> homoArray;
		    for(int i=0; i!=3; i++){
			    for(int j=0; j!=3; j++){
				    homoArray.push_back(homoMat.at<double>(i, j));
			    }
		    }
		    
		    // Set homography matrix as rosParam.
		    nh.setParam("homography/data", homoArray);

		    /*
		    nh.setParam("homography/matrix_x1", homoMat.at<double>(0, 0));
		    nh.setParam("homography/matrix_y1", homoMat.at<double>(1, 0));
		    nh.setParam("homography/matrix_z1", homoMat.at<double>(2, 0));
		    nh.setParam("homography/matrix_x2", homoMat.at<double>(0, 1));
		    nh.setParam("homography/matrix_y2", homoMat.at<double>(1, 1));
		    nh.setParam("homography/matrix_z2", homoMat.at<double>(2, 1));
		    nh.setParam("homography/matrix_x3", homoMat.at<double>(0, 2));
		    nh.setParam("homography/matrix_y3", homoMat.at<double>(1, 2));
		    nh.setParam("homography/matrix_z3", homoMat.at<double>(2, 2));
		    */
		    std::vector<double> intrinsicArray, distortionArray;
		    nh.getParam("usb_cam/camera_matrix/data", intrinsicArray);
		    nh.getParam("usb_cam/distortion_coefficients/data", distortionArray);

		    cv::Mat intrinsic(3, 3, CV_64F);

		    intrinsic.at<double>(0, 0) = intrinsicArray[0];
		    intrinsic.at<double>(0, 1) = intrinsicArray[1];
		    intrinsic.at<double>(0, 2) = intrinsicArray[2];
		    intrinsic.at<double>(1, 0) = intrinsicArray[3];
		    intrinsic.at<double>(1, 1) = intrinsicArray[4];
		    intrinsic.at<double>(1, 2) = intrinsicArray[5];
		    intrinsic.at<double>(2, 0) = intrinsicArray[6];
		    intrinsic.at<double>(2, 1) = intrinsicArray[7];
		    intrinsic.at<double>(2, 2) = intrinsicArray[8];

		    cv::Mat distortion(4, 1, CV_64F);

		    distortion.at<double>(0, 0) = distortionArray[0];
		    distortion.at<double>(0, 1) = distortionArray[1];
		    distortion.at<double>(0, 2) = distortionArray[2];
		    distortion.at<double>(0, 3) = distortionArray[3];

  		    std::vector<double> RArray, camPose;
		    
		    extrinsicCalibration(arucoEdge, intrinsic, distortion);
  
	  	    for(int i=0; i!=3; i++){
		  	    for(int j=0; j!=3; j++){
			  	  RArray.push_back(R.at<double>(i, j));
		    	    }
	  	    }

	  	    //for(int i=0; i!=3; i++)
		  //	    camPose.push_back(cam_pos.at<double>(0, i));
	  
		    camPose.push_back(pos_x);
		    camPose.push_back(pos_y);
		    camPose.push_back(pos_z);


		    // Set R, yaw, pitch, roll and camera position as rosParam
		    nh.setParam("usb_cam/extrinsic/R", RArray);
		    nh.setParam("usb_cam/extrinsic/yaw", yaw);
	  	    nh.setParam("usb_cam/extrinsic/pitch", pitch);
	  	    nh.setParam("usb_cam/extrinsic/roll", roll);
		    nh.setParam("usb_cam/extrinsic/pose", camPose);
	    }
			
    }
  }
 
  // function to publish homography transformed image. 
  void transformCallback(const sensor_msgs::ImageConstPtr &msg){
	
	// CvImagePtr structure
	// std_msgs::Header header;
	// std::string encoding;
	// cv::Mat image;
	//
	// --> use cv_ptr->image access image message on ROS.
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImage img_bridge;
	sensor_msgs::Image img_msg;


	cv::Mat img;
	cv::Size size(width, height);
	std_msgs::Header header;
	
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);


	std::cout << homoMat << std::endl << std::endl;
	cv::warpPerspective(cv_ptr->image, img, homoMat, size);

	header.stamp = ros::Time::now();
	
	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);

	img_bridge.toImageMsg(img_msg);
	img_pub.publish(img_msg);
  }

private:
  ros::NodeHandle nh; 
  ros::Subscriber sub;
  
  image_transport::ImageTransport it;
  image_transport::Publisher img_pub;
  image_transport::Subscriber img_sub;
		    
  double yaw, pitch, roll;
  double pos_x, pos_y, pos_z;

  cv::Mat homoMat = cv::Mat::zeros(3, 3, CV_64F);
  cv::Mat R, cam_pos;
  
  int width=800;
  int height=800;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscribe_and_img_publish");

  homographyTransform SAPObject;

  ros::spin();

  return 0;
}
