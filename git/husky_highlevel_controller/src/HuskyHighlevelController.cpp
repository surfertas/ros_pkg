#include "husky_highlevel_controller/HuskyHighlevelController.hpp"
#include <cmath>

namespace husky_highlevel_controller {

    HuskyHighlevelController::HuskyHighlevelController(ros::NodeHandle& nodeHandle) :
      nh_(nodeHandle)
    {
        
        if (!readParameters()) {
            ROS_ERROR("Could not read parameter.");
            ros::requestShutdown();
        }
 
        sub_ = nh_.subscribe(subscribeTopic_, Qsize_, &HuskyHighlevelController::topicCB, this);
        pub_ = nh_.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", Qsize_);
        vis_pub_ = nh_.advertise<visualization_msgs::Marker>("/husky_laserscan/visualization_marker", 0);

        ROS_INFO("Node launched.");
    }

    HuskyHighlevelController::~HuskyHighlevelController()
    {
    }
    
    int HuskyHighlevelController::readParameters()
    {
        return (nh_.getParam("topic_name", subscribeTopic_)
                && nh_.getParam("queue_size", Qsize_));
    }

    void HuskyHighlevelController::topicCB(const sensor_msgs::LaserScan& msg)
    {
        float min = INFINITY;   //Min value in range.
        float theta = 0.0;      //Turn angle.
        float d_init = 0.0;     //Initial distance to pillar.         
        auto min_idx = 0;       //Index of distance of pillar.
        auto len = msg.ranges.size();
        
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
        auto vel = [](float x) -> float { if (x < 0.5)
                                            return 0.0;
                                          return x; } (min);

      
        cmd_msg.linear.x = vel;
        cmd_msg.linear.y = vel;
        pub_.publish(cmd_msg);
      
        //TODO: calculate location of pillar and pass to pillarMarker          
        auto x_coord = min*cos(-theta);
        auto y_coord = min*sin(-theta);

        pillarMarker(x_coord, y_coord);
        ROS_INFO("Distance to Pillar: %f", min); 
    }
    

    void HuskyHighlevelController::pillarMarker(double x, double y)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_laser";
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
        vis_pub_.publish(marker);
    }


} /* namespace */
