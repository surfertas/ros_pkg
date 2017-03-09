#pragma once

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

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
         * Subscriber callback to calculate the shortest range
         * measurement.
         * @arg: msg - contains info related to LaserScan.msg.
         */
        void topicCB(const sensor_msgs::LaserScan& msg);

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        std::string subscribeTopic_;
        int Qsize_;
    };

} /* namespace */
