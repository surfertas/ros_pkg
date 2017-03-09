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
        float min = INFINITY;
        uint32_t len = sizeof(msg.ranges)/sizeof(msg.ranges[0]);

        for (auto val: msg.ranges) {
            if (min == INFINITY) 
                min = val;

            if (val < min)
                min = val;
        }
        ROS_INFO("range value: %f", min); 
    }
} /* namespace */
