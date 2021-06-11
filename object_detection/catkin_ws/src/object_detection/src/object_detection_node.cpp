#include <stdlib.h>
#include <memory>
#include <ros/ros.h>
#include <object_detection/object_detector.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

class ObjectDetectionNode : public ObjectDetector
{
public:
    ObjectDetectionNode(const std::string &classes_filename, const std::string &model_filename, const std::string &model_config,
                        float score_threshold, ros::NodeHandle *nh) : ObjectDetector(classes_filename, model_filename, model_config, score_threshold), nh_(nh), it_(*nh)
    {
        // ROS_INFO("OD Object");
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
        // ROS_INFO("Image cb");

        std::vector<Prediction> predictions = ProcessFrame(cv_ptr->image);
        // ROS_INFO("Image cb");

        // Update GUI Window
        cv::imshow("Image window", cv_ptr->image);
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
    ros::NodeHandle *nh_; // ROS node handler
    image_transport::Publisher image_pub_;
    image_transport::Subscriber image_sub_;
    cv_bridge::CvImage bridge;
    image_transport::ImageTransport it_;

};

main(int argc, char **argv)
{
    // ROS stuff
    ros::init(argc, argv, "od_node");
    ros::Time::init();
    ros::NodeHandle n("~");
    ros::Rate r(10.0);

    std::string class_filename = "/home/developer/object_detection/catkin_ws/src/object_detection/model/object_detection_classes_coco.txt";
    std::string model_filename = "/home/developer/object_detection/catkin_ws/src/object_detection/model/frozen_inference_graph.pb";
    std::string model_config = "/home/developer/object_detection/catkin_ws/src/object_detection/model/ssd_mobilenet_v2_coco_2018_03_29.pbtxt.txt";
    // ROS_INFO("creating object");
    ObjectDetectionNode object_detection_node(class_filename, model_filename, model_config, 0.4, &n);

    while (n.ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}