#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <iostream>
#include "boost/filesystem.hpp"
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <boost/regex.hpp>

using namespace std;
using namespace boost::filesystem;
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  string image_dir;

  if (!nh.getParam("image_directory", image_dir))
  {
    ROS_ERROR("Image directory (parameter: image_directory) not specified, exiting");
  }
  else
  {
    ROS_INFO("Reading images from: %s", image_dir.c_str());
  }
 
  path p(image_dir); 
  directory_iterator end_itr;
 
  std::vector<std::string> accumulator;
  // cycle through the directory
  for (directory_iterator itr(p); itr != end_itr; ++itr)
  {
    // If it's not a directory, list it. If you want to list directories too, just remove this check.
    if (is_regular_file(itr->path())) {
      // assign current file name to current_file and echo it out to the console.
      string current_file = itr->path().string();
      if( itr->path().extension().string() == ".pgm" )    
      {
        cout << itr->path().string() << endl;
        accumulator.push_back(itr->path().string());
      }
    }
  }
  
  std::sort(accumulator.begin(), accumulator.end());
  std::vector<std::string>::iterator iter;
  for (iter = accumulator.begin(); iter != accumulator.end(); ++iter) {
    std::cout << "--> considering file " << *iter << "... \n"; 
  }

  int i=0;
  ros::Rate loop_rate(5);
  while (nh.ok()) {
    cv::Mat image;
    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
    ROS_INFO("Publishing: %s", accumulator[i].c_str());
    image = cv::imread(accumulator[i++], CV_LOAD_IMAGE_GRAYSCALE); 
    cv_ptr->image = image;
    sensor_msgs::ImagePtr img_msg = cv_ptr->toImageMsg();
    pub.publish(img_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
