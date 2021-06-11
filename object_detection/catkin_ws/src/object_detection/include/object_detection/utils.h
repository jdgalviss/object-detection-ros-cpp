#include <ros/ros.h>
#include <string>

template <class class_name>
bool getParameter(const std::string& paramName, class_name& param)
{
    std::string nodeName = ros::this_node::getName();
    std::string paramKey;
    if (!ros::param::search(paramName, paramKey))
    {
        ROS_ERROR("%s: Failed to search for parameter '%s'.", nodeName.c_str(), paramName.c_str());
        return false;
    }

    if (!ros::param::has(paramKey))
    {
        ROS_ERROR("%s: Missing required parameter '%s'.", nodeName.c_str(), paramName.c_str());
        return false;
    }

    if (!ros::param::get(paramKey, param))
    {
        ROS_ERROR("%s: Failed to get parameter '%s'.", nodeName.c_str(), paramName.c_str());
        return false;
    }

    return true;
}