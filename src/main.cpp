#include <ros/ros.h>

#include "MyAlgorithm.h"

#include <iostream>
#include <sys/time.h>
#include <Eigen/Dense>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/image_encodings.h>

bool image_updated     = false;
bool algorithm_updated = false;

typedef std::string TopicTimeStr;
typedef double TopicTime;

TopicTimeStr image_time_str;
TopicTime image_time;

cv::Mat current_image;

inline std::string dtos(double x){
	std::stringstream s;
	s<<std::setprecision(6) << std::fixed << x;
	return s.str();
}

void image_callback(const sensor_msgs::ImageConstPtr& msg){
	cv_bridge::CvImagePtr cv_ptr;
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
	current_image = cv_ptr->image;
  	image_time = (double)(msg->header.stamp.sec*1e6+msg->header.stamp.nsec/1000)/1000000.0;
	image_time_str = dtos(image_time);
	image_updated = true;
	ROS_INFO_STREAM("SUBSCRIBER : Image updated");
}

int main(int argc, char **argv){

	ros::init(argc, argv, "realtime_node");
	ros::NodeHandle nh("~");

    	std::string image_topic_name;
        bool debug_flag;
    	ros::param::get("~image_topic_name", image_topic_name);
	ros::param::get("~debug_image",debug_flag);

	ros::Subscriber image_sub;
	image_sub = nh.subscribe<sensor_msgs::Image>(image_topic_name,10,&image_callback);

	MyAlgorithm::Parameters parameters;
	parameters.debug_flag.print_image = debug_flag;

	MyAlgorithm *my_algorithm = new MyAlgorithm(parameters);

	while(ros::ok()) {
		ros::spinOnce(); // so fast
		if(image_updated == true) {
			my_algorithm->image_acquisition(current_image,image_time);
			my_algorithm->run();
			image_updated = false;
		}
	}

	delete my_algorithm;
	return 0;
}
