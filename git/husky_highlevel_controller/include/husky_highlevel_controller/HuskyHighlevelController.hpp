#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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
        HuskyHighlevelController(ros::NodeHandle& nodeHandle);

        /*
         * Destructor.
         */
        virtual ~HuskyHighlevelController();
    
        /*
         * Checks if specified parameters are available.
         * @rets: returns 1 if all parameters found, else 0        
         */
        int readParameters();

        /*
         * Subscriber callback to calculate the distance to pillar,
         * and publishes geometry_msg::Twist to implement a 
         * proportional controller.
         * @arg: msg - contains info related to LaserScan.msg.
         */
        void topicCB(const sensor_msgs::LaserScan& msg);

        /* Defines location and related specification of marker
         * used to represent the pillar.
         * @arg: x - x coordinate of pillar.
         * @arg: y - y coordinate of pillar.
         */
        void pillarMarker(double x, double y);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        ros::Publisher vis_pub_;
        tf2_ros::Buffer tfBuffer_;
        tf2_ros::TransformListener listener_ {tfBuffer_}; 
        std::string subscribeTopic_;
        int Qsize_;
    };

} /* namespace */
