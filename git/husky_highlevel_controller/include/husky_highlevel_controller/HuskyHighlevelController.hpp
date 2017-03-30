// This file is the header to HuskyHighlevelController.cpp.
//
// Work based off the open source course, Programming for Robotics - ROS
// by ETZH (http://www.rsl.ethz.ch/education-students/lectures/ros.html)
// Date:    3/29/2017
// Author:  Tasuku Miura

#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/SetBool.h>
#include <visualization_msgs/Marker.h>

namespace husky_highlevel_controller {

    /*
     *Class containing the Husky Highlevel Controller
     */
    class HuskyHighlevelController {
    public:
        /*
         *Constructor.
         */
        HuskyHighlevelController(ros::NodeHandle&, bool manual_control);

        /*
         * Destructor.
         */
        virtual ~HuskyHighlevelController();
    
    private:
        void registerService();
        void registerSubscriber();
        void registerPublisher();

        /*
         * Service that sets husky_manual_control, which allows
         * user to stop/start husky from command line using rosservice 
         * call.
         * @args: req - request defined in std_srvs::SetBool.
         * @args: resp - response as defined in std_srvs::SetBool.
         * @rets: returns true on success.        
         */
        bool controlCB(std_srvs::SetBool::Request &req,
                       std_srvs::SetBool::Response &resp);

        /*
         * Subscriber callback to calculate the distance to pillar,
         * and publishes geometry_msg::Twist to implement a 
         * proportional controller.
         * @args: msg - contains info related to LaserScan.msg.
         */
        void topicCB(const sensor_msgs::LaserScan& msg);

        /* Defines location and related specification of marker
         * used to represent the pillar.
         * @args: x - x coordinate of pillar.
         * @args: y - y coordinate of pillar.
         */
        void pillarMarker(double x, double y);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_laser_scan_;
        ros::Publisher pub_husky_twist_;
        ros::Publisher pub_visualization_marker_;
        
        ros::ServiceServer service_manual_control_;
        bool husky_manual_control_;
    
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener listener_ {tfBuffer_}; 
        ros::ServiceClient client_;
    };

} /* namespace */
