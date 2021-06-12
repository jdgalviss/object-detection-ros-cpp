#include <stdlib.h>
#include <memory>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "object_detection/object_detector.h"
#include "object_detection/utils.h"


class ObjectDetectionNode : public ObjectDetector
{
public:
    ObjectDetectionNode(const std::string &classes_filename, const std::string &model_filename, const std::string &model_config,
                        float score_threshold, bool display_image, ros::NodeHandle *nh) : ObjectDetector(classes_filename, model_filename, model_config, 
                        score_threshold), display_image_(display_image), nh_(nh), it_(*nh)
    {
        InitializePubSub();
    }

private:
    void InitializePubSub()
    {
        image_pub_ = it_.advertise("object_detection/image", 1);
        image_sub_ = it_.subscribe("/cv_camera/image_raw", 1, &ObjectDetectionNode::ImageCb, this);
    }
    void ImageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        ROS_INFO("Image cb");
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        std::vector<Prediction> predictions = ProcessFrame(cv_ptr->image);

        // Update GUI Window
        if(display_image_){
            cv::imshow("Image window", cv_ptr->image);
            cv::waitKey(3);
        }
        image_pub_.publish(cv_ptr->toImageMsg());
    }
    bool display_image_;
    ros::NodeHandle *nh_; // ROS node handler
    image_transport::Publisher image_pub_;
    image_transport::Subscriber image_sub_;
    cv_bridge::CvImage bridge;
    image_transport::ImageTransport it_;
};

int main(int argc, char **argv)
{
    // ROS stuff
    ros::init(argc, argv, "od_node");
    ros::Time::init();
    ros::NodeHandle n("~");
    ros::Rate r(10.0);
    std::string class_filename;
    std::string model_filename;
    std::string model_config;
    float score_threshold;
    bool display_image;

    // Read Parameters
    if (!getParameter("/model/class_filename", class_filename))
    {
        ROS_ERROR("class_filename not set");
        return -1;
    }

    if (!getParameter("/model/model_filename", model_filename))
    {
        ROS_ERROR("model_filename not set");
        return -1;
    }

    if (!getParameter("/model/config_file", model_config))
    {
        ROS_ERROR("model_config not set");
        return -1;
    }

    if (!getParameter("/model/score_threshold", score_threshold))
    {
        ROS_WARN("score_threshold not set, using default value: 0.4");
        score_threshold = 0.5;
    }

    if (!getParameter("/display/imshow", display_image))
    {
        ROS_WARN("display_image not set, using default value: false");
        display_image = false;
    }

    std::unique_ptr<ObjectDetectionNode> object_detection_node;
    try
    {
        object_detection_node = std::make_unique<ObjectDetectionNode>(class_filename, model_filename, model_config, score_threshold, display_image, &n);
    }
    catch (std::runtime_error e)
    {
        std::cout << e.what() 
        << std::endl;
    }

    while (n.ok())
    {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}