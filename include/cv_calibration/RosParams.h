#ifndef __ROSPARAMS_H__
#define __ROSPARAMS_H__

#include <string>
#include "ros/ros.h"

struct RosParams
{
    std::string robot_name;
    std::string robot_ip;
    std::string color_address;
    std::string depth_address;
    bool readArgs()
    {
        bool success = true;
        success &= ros::param::get("~robot_name", robot_name);
        success &= ros::param::get("~robot_ip", robot_ip);
        success &= ros::param::get("~color_address", color_address);
        success &= ros::param::get("~depth_address", depth_address);
        return success;
    }
};

#endif // __ROSPARAMS_H__