#ifndef _MY_ALGORITHM_H_
#define _MY_ALGORITHM_H_

#include <ros/ros.h>

#include <iostream>
#include <sys/time.h>
#include <Eigen/Dense>
#include <ctime>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#define PI 3.141592

typedef std::string TopicTimeStr;
typedef double TopicTime;

class MyAlgorithm{

public:

 struct Calibration {
    double fx;  //  focal length x
    double fy;  //  focal length y
    double cx;  //  principal point (u-coordinate)
    double cy;  //  principal point (v-coordinate)
    int width;
    int height; 
    Calibration () {
      fx     = 620.608832234754;
      fy     = 619.113993685335;
      cx     = 323.902900972212;
      cy     = 212.418428046497;
      width  = 752;
      height = 480;
    }
  };

  // debug parameters
  struct DebugFlag {
    bool print_image;
    DebugFlag () {
      print_image = false;
    }
  };
  
  struct FeatureDetector { 
    int max_features;
    double quality_level;
    double min_distance;
    int block_size;
    bool use_Harris_detector;
    double k;
    FeatureDetector () {
	max_features = 250;
 	quality_level = 0.01;
   	min_distance  = 10;
        block_size    = 4;
	use_Harris_detector = false;
	k = 0.04;
    }
  };

  struct FeatureTracker { 
    int win_size;
    int max_level;
    int flags_R;
    FeatureTracker () {
	win_size = 11;
	max_level = 5;
	flags_R = 0;
    }
  };

  struct Parameters { 
    MyAlgorithm::Calibration calibration;
    MyAlgorithm::DebugFlag debug_flag;
    MyAlgorithm::FeatureDetector feature_detector;
    MyAlgorithm::FeatureTracker feature_tracker;
  };

  bool complete_flag;

  MyAlgorithm(Parameters parameters);
  ~MyAlgorithm();
  void image_acquisition(const cv::Mat& img,const TopicTime& curr_time);
  void run();
  void point_plot(cv::Mat& result_image, const std::vector<cv::Point2f>& curr_features);

private:
  Parameters my_param;
  TopicTime curr_time;

  cv::Mat curr_image;
  cv::Mat prev_image;

  std::vector<cv::Point2f> curr_features;
  std::vector<cv::Point2f> prev_features;
  
  std::vector<cv::Point2f> curr_features_valid;
  std::vector<cv::Point2f> prev_features_valid;

  int num_feature_valid;
  
};
#endif
