// Work based off the open source course, Programming for Robotics - ROS
// by ETZH (http://www.rsl.ethz.ch/education-students/lectures/ros.html)
// Date:    3/29/2017
// Author:  Tasuku Miura

#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <cmath>

namespace husky_highlevel_controller {

    HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle,
                                                       bool manual_control) :
        nh_(nodeHandle),
        husky_manual_control_(manual_control)
    {
        registerSubscriber();
        registerService();
        registerPublisher(); 
        ROS_INFO("Node launched.");
    }

    HuskyHighlevelController::~HuskyHighlevelController()
    {
    }
    
    void HuskyHighlevelController::registerService()
    {
        service_manual_control_ =
            nh_.advertiseService("manual_control_override", &HuskyHighlevelController::controlCB, this);
    }

    void HuskyHighlevelController::registerSubscriber()
    {
        sub_laser_scan_ = 
            nh_.subscribe("/scan", 10, &HuskyHighlevelController::topicCB, this);
    }

    void HuskyHighlevelController::registerPublisher()
    {
        pub_husky_twist_ =
            nh_.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 10);
        pub_visualization_marker_ = 
            nh_.advertise<visualization_msgs::Marker>("/husky_laserscan/visualization_marker", 0);
    }

    bool HuskyHighlevelController::controlCB(std_srvs::SetBool::Request &req,
                                             std_srvs::SetBool::Response &resp)
    {
        husky_manual_control_ = req.data;
        resp.success = true;
        return resp.success;
    }

    void HuskyHighlevelController::topicCB(const sensor_msgs::LaserScan& msg)
    {
        float min = INFINITY;   //Min value in range.
        float theta = 0.0;      //Turn angle.
        float d_init = 0.0;     //Initial distance to pillar.         
        auto min_idx = 0;       //Index of distance of pillar.
        auto len = msg.ranges.size();
      
        geometry_msgs::TransformStamped transformStamped;
        try {
            transformStamped = tfBuffer_.lookupTransform("odom", "base_laser",
                                                         ros::Time(0));
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        for (auto i = 0; i < len; i++) {
            if (min == INFINITY) 
                min = msg.ranges[i];

            if (msg.ranges[i] < min) {
                min = msg.ranges[i];
                min_idx = i;
            }
        }

        geometry_msgs::Twist cmd_msg;
        
        //Calculates turn angle, left hand rule.
        theta = msg.angle_min + min_idx * msg.angle_increment;
        cmd_msg.angular.z = -theta;

        //Stops Husky from crashing into pillar. 
        if (min < 0.5)
            husky_manual_control_ = true;    
        
        //Check if there has been trigger to stop Husky.
        if (!husky_manual_control_) {
            cmd_msg.linear.x = min;
            cmd_msg.linear.y = min;
        } else {
            cmd_msg.linear.x = 0.0;
            cmd_msg.linear.y = 0.0;
        }
        pub_husky_twist_.publish(cmd_msg);
      
        geometry_msgs::PoseStamped pose_in;
        geometry_msgs::PoseStamped pose_out;

        pose_in.pose.position.x = min*cos(-theta);
        pose_in.pose.position.y = min*sin(-theta);
        pose_in.header.stamp = ros::Time(0);
        pose_in.header.frame_id = "base_laser";
        tfBuffer_.transform(pose_in, pose_out, "odom");
        
        pillarMarker(pose_out.pose.position.x, 
                     pose_out.pose.position.y);

        ROS_INFO("Distance to Pillar: %f", min); 
    }
    
    void HuskyHighlevelController::pillarMarker(double x, double y)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time();
        marker.ns = "husky_highlevel_controller";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        pub_visualization_marker_.publish(marker);
    }

} /* namespace */
