// Work based off the open source course, Programming for Robotics - ROS
// by ETZH (http://www.rsl.ethz.ch/education-students/lectures/ros.html)
// Date:    3/29/2017
// Author:  Tasuku Miura

#include <ros/ros.h>
#include "husky_highlevel_controller/HuskyHighlevelController.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "husky_highlevel_controller");
  ros::NodeHandle nodeHandle("~");

  husky_highlevel_controller::HuskyHighlevelController huskyHighlevelController(nodeHandle, false);

  ros::spin();
  return 0;
}
