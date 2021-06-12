#include <ros/ros.h>
#include <string>

template <class class_name>
bool getParameter(const std::string& param_name, class_name& param)
{
    std::string node_name = ros::this_node::getName();
    std::string param_key;
    if (!ros::param::search(param_name, param_key))
    {
        ROS_ERROR("%s: Failed to search for parameter '%s'.", node_name.c_str(), param_name.c_str());
        return false;
    }

    if (!ros::param::has(param_key))
    {
        ROS_ERROR("%s: Missing required parameter '%s'.", node_name.c_str(), param_name.c_str());
        return false;
    }

    if (!ros::param::get(param_key, param))
    {
        ROS_ERROR("%s: Failed to get parameter '%s'.", node_name.c_str(), param_name.c_str());
        return false;
    }

    return true;
}