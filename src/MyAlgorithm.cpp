#include "MyAlgorithm.h"

#include <string>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>

MyAlgorithm::MyAlgorithm(Parameters parameters):my_param(parameters){
  if(my_param.debug_flag.print_image == true) cv::namedWindow("current image");
  this->complete_flag = true;
}

MyAlgorithm::~MyAlgorithm(){
  ROS_INFO_STREAM("Realtime node is terminated.\n");
}


void MyAlgorithm::point_plot(cv::Mat& result_image, const std::vector<cv::Point2f>& curr_features){
  int width = my_param.calibration.width;
  int height = my_param.calibration.height;
  cv::Scalar line_color = CV_RGB(0,255,255);

  std::cout<<curr_features.size()<<std::endl;

  for(int i=0; i < (int)curr_features.size(); i++){
     cv::Point2f p;
     p = curr_features[i];
     cv::circle(result_image, p, 6,line_color);
  }
}


void MyAlgorithm::image_acquisition(const cv::Mat& img,const TopicTime& curr_time_input){
  this->curr_image = img.clone();
  this->curr_time  = curr_time_input;
  ROS_INFO_STREAM("In algorithm image is updated.");
}

void MyAlgorithm::run(){

  //cv::goodFeaturesToTrack(curr_image,curr_features,my_param.feature_detector.max_features, my_param.feature_detector.quality_level, my_param.feature_detector.min_distance, cv::Mat(), my_param.feature_detector.block_size, my_param.feature_detector.use_Harris_detector, my_param.feature_detector.k);

  cv::Size patternsize(6,4);
  cv::findChessboardCorners(curr_image,patternsize, curr_features);

  if(curr_features.size()>0) cv::cornerSubPix(curr_image, curr_features, cv::Size(7,7), cv::Size(-1,-1), cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
  
  if(my_param.debug_flag.print_image == true){
	cv::Mat result_image;
	cv::cvtColor(curr_image, result_image, cv::COLOR_GRAY2RGB);
        if(curr_features.size()>0) this->point_plot(result_image,curr_features);
	cv::imshow("current image",result_image);
 	cv::waitKey(1);
  	result_image.release();
  }
  // algorithm execution in this function.
  ROS_INFO_STREAM("   algorithm is operated.");

  this->complete_flag = true;
}
