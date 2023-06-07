#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "fiducial_msgs/FiducialArray.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float64MultiArray.h"
#include <iostream>
#include <vector>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    pub_ = n_.advertise<std_msgs::Float64MultiArray>("/homography", 1);
    sub_ = n_.subscribe("/fiducial_vertices", 100, &SubscribeAndPublish::callback, this);
  }

  void callback(const fiducial_msgs::FiducialArray::ConstPtr& msg)
  {
    std_msgs::Float64MultiArray homo;
    cv::Mat homoMat;


    int len = msg->fiducials.size();

    if(len>=4){
	    int id, isFour=0;
	    std::vector<cv::Point2f> arucoEdge, perspectiveEdge;
	    cv::Point a1, a2, a3, a4, p1, p2, p3, p4;

    	    for(int i=0; i!=len; i++){
	    	id = msg->fiducials[i].fiducial_id;
	    	//Detect edge markers. It's possible to change id.
	    	switch(id){
			    case 43:
				    a1=cv::Point2f(msg->fiducials[i].x0, msg->fiducials[i].y0);
			      	    isFour++;
			    	    break;
		    	    case 5:
			    	    a2=cv::Point2f(msg->fiducials[i].x0, msg->fiducials[i].y0);
			    	    isFour++;
			    	    break;
		    	    case 12:
			    	    a3=cv::Point2f(msg->fiducials[i].x0, msg->fiducials[i].y0);
			    	    isFour++;
			    	    break;
		    	    case 18:
				    a4=cv::Point2f(msg->fiducials[i].x0, msg->fiducials[i].y0);
				    isFour++;
				    break;
		}
	    }

	    if(isFour==4){
		    int width=1000;
		    int height=1000;

		    p1=cv::Point2f(0, 0);
		    p2=cv::Point2f(width, 0);
		    p3=cv::Point2f(0, height);
		    p4=cv::Point2f(width, height);

		    std::vector<cv::Point2f> arucoEdge={a1, a2, a3, a4};
		    std::vector<cv::Point2f> perspectiveEdge={p1, p2, p3, p4};

		    homoMat = cv::findHomography(arucoEdge, perspectiveEdge);

		    for(int i=0; i!=homoMat.rows; i++){
			    for(int j=0; j!=homoMat.cols; j++){
				   homo.data.push_back(homoMat.at<float>(i, j));
			    }
		    }
	    }
    }
    pub_.publish(homo);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscribe_and_publish");

  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
