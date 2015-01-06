#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <iostream>
#include "boost/filesystem.hpp"
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <boost/regex.hpp>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <camera_calibration_parsers/parse_yml.h>
#include <camera_calibration_parsers/parse_yml.h>
using namespace std;
using namespace boost::filesystem;
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh; 
  
  string left_camera_name = "camera_left";
  string right_camera_name = "camera_right";
 
  string left_frame_id_;
  string right_frame_id_;

  string regex_left_str_;
  string regex_right_str_;

  double publish_rate_;

  if (!nh.getParam("left_frame_id", left_frame_id_))
    left_frame_id_ = "/camera/left";
 
  if (!nh.getParam("right_frame_id", right_frame_id_))
    left_frame_id_ = "/camera/right";
 
 
  ros::NodeHandle nh_left(left_frame_id_);
  ros::NodeHandle nh_right(right_frame_id_);
  image_transport::ImageTransport it_left(nh_left);
  image_transport::ImageTransport it_right(nh_right);
  image_transport::CameraPublisher pub_left = it_left.advertiseCamera("image", 1);
  image_transport::CameraPublisher pub_right = it_right.advertiseCamera("image", 1);
  string image_dir;
  string left_calibration_file_;
  string right_calibration_file_;
 
  camera_info_manager::CameraInfoManager cinfo_left_(nh_left);
  cinfo_left_.setCameraName(left_camera_name);
  camera_info_manager::CameraInfoManager cinfo_right_(nh_right);
  cinfo_right_.setCameraName(right_camera_name);

  if (!nh.getParam("image_directory", image_dir))
  {
    ROS_ERROR("Image directory (parameter: image_directory) not specified, exiting");
    exit(0);
  }

  if (!nh.getParam("left_calibration_file", left_calibration_file_))
  {
    ROS_ERROR("Left calibration file (parameter: left_calibration_file) not specified, exiting");
    exit(0);
  }

  if (!nh.getParam("right_calibration_file", right_calibration_file_))
  {
    ROS_ERROR("Right calibration file (parameter: right_calibration_file) not specified, exiting");
    exit(0);
  }

  if(!nh.getParam("regex_left", regex_left_str_))
  {
    ROS_ERROR("Regex for left image files not supplied. Exiting");
    exit(0);
  }
  else
    ROS_INFO("Applying %s regex to left image files", regex_left_str_.c_str());

  if(!nh.getParam("regex_right", regex_right_str_))
  {
    ROS_ERROR("Regex for right image files not supplied. Exiting");
    exit(0);
  }
  if(!nh.getParam("publish_rate", publish_rate_))
    publish_rate_ = 10;

  if(!cinfo_left_.validateURL(left_calibration_file_))
  {
    ROS_ERROR("Could not read calibration from %s, exiting", left_calibration_file_.c_str());
    exit(0);
  }
  
  if(!cinfo_left_.loadCameraInfo(left_calibration_file_))
  {
    ROS_ERROR("Can't load calibration file: exiting");
    exit(0);
  }
  
  if(!cinfo_right_.validateURL(right_calibration_file_))
  {
    ROS_ERROR("Could not read calibration for %s, exiting", right_camera_name.c_str());
    exit(0);
  }
  if(!cinfo_right_.loadCameraInfo(right_calibration_file_))
  {
    ROS_ERROR("Can't load calibration file: exiting");
    exit(0);
  }

  path p(image_dir); 
  directory_iterator end_itr;
 
  std::vector<std::string> left_accumulator;
  std::vector<std::string> right_accumulator;
  boost::regex left_name(regex_left_str_.c_str());
  boost::regex right_name(regex_right_str_.c_str());
  boost::cmatch what;

  // cycle through the directory
  for (directory_iterator itr(p); itr != end_itr; ++itr)
  {
    // If it's not a directory, list it. If you want to list directories too, just remove this check.
    if (is_regular_file(itr->path())) {
      // assign current file name to current_file and echo it out to the console.
      string current_file = itr->path().filename().string();
      if(boost::regex_match(current_file, left_name))
      {
        left_accumulator.push_back(itr->path().string());
      }
      else if(boost::regex_match(current_file, right_name))
      {
        right_accumulator.push_back(itr->path().string());
      }
    }
  }
  
  std::sort(left_accumulator.begin(), left_accumulator.end());
  std::sort(right_accumulator.begin(), right_accumulator.end());

  int i=0;
  ros::Rate loop_rate(10);
  while (nh.ok()) {
    ROS_INFO("Publishing images");
    cv::Mat image_left, image_right;
    cv_bridge::CvImagePtr cv_ptr_left(new cv_bridge::CvImage);
    cv_bridge::CvImagePtr cv_ptr_right(new cv_bridge::CvImage);
    image_left = cv::imread(left_accumulator[i], CV_LOAD_IMAGE_GRAYSCALE); 
    image_right = cv::imread(right_accumulator[i++], CV_LOAD_IMAGE_GRAYSCALE); 
    if(i == left_accumulator.size()) 
    {
      std::cout << "Out of images, exiting\n";
      exit(0);
    }
    cv_ptr_left->image = image_left;
    cv_ptr_right->image = image_right;
    cv_ptr_left->encoding = sensor_msgs::image_encodings::MONO8;
    cv_ptr_right->encoding = sensor_msgs::image_encodings::MONO8;
    sensor_msgs::ImagePtr img_msg_left = cv_ptr_left->toImageMsg();
    sensor_msgs::ImagePtr img_msg_right = cv_ptr_right->toImageMsg();
    ros::Time tNow = ros::Time::now();
    img_msg_left->header.stamp = tNow;
    img_msg_left->header.frame_id = left_frame_id_;
    img_msg_right->header.frame_id = right_frame_id_;

    img_msg_right->header.stamp= tNow;
    
    // get current CameraInfo data
    sensor_msgs::CameraInfo ci_left = cinfo_left_.getCameraInfo();
    ci_left.header.frame_id = left_frame_id_;
    sensor_msgs::CameraInfo ci_right = cinfo_right_.getCameraInfo();
    ci_right.header.frame_id = right_frame_id_;

    ci_left.header.stamp = tNow;
    ci_right.header.stamp= tNow;
   
    pub_left.publish(*img_msg_left, ci_left, tNow);
    pub_right.publish(*img_msg_right, ci_right, tNow);
   
    ros::spinOnce();
    loop_rate.sleep();
  }
}
