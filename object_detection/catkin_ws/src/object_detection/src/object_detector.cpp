#include "object_detection/object_detector.h"

ObjectDetector::ObjectDetector(const std::string &classes_filename, const std::string &model_filename, 
                                const std::string &model_config, float score_threshold)
{
    score_threshold_ = score_threshold;
    std::ifstream ifs(classes_filename.c_str());
    if(!ifs){
        throw std::runtime_error("Classes filename doesn't exist");
    }
    std::string line;
    while (getline(ifs, line))
    {
        class_names_.push_back(line);
    }  
    ifs.close();
    model_ = cv::dnn::readNet(model_filename, model_config, "Tensorflow");
}


std::vector<Prediction> ObjectDetector::ProcessFrame(cv::Mat &image){
    image_height_ = image.cols;
    image_width_ = image.rows;
    cv::Mat blob = cv::dnn::blobFromImage(image, 1.0, cv::Size(300, 300), cv::Scalar(127.5, 127.5, 127.5), 
                            true, false);
    model_.setInput(blob);
    cv::Mat output = model_.forward();
    cv::Mat detectionMat(output.size[2], output.size[3], CV_32F, output.ptr<float>());
    std::vector<Prediction> result;
    for (int i = 0; i < detectionMat.rows; i++){
        Prediction pred;
        pred.label = detectionMat.at<float>(i, 1);
        pred.score = detectionMat.at<float>(i, 2);
        if(pred.score > score_threshold_){
            int box_x = static_cast<int>(detectionMat.at<float>(i, 3) * image.cols);
            int box_y = static_cast<int>(detectionMat.at<float>(i, 4) * image.rows);
            int box_width = static_cast<int>(detectionMat.at<float>(i, 5) * image.cols - box_x);
            int box_height = static_cast<int>(detectionMat.at<float>(i, 6) * image.rows - box_y);
            pred.box = std::vector<int>{box_x, box_y, box_width, box_height};
            DrawBoxes(image, pred);
            result.push_back(pred);
        }
    }
    return result;
}

void ObjectDetector::DrawBoxes(cv::Mat &image, Prediction pred)
{
    cv::rectangle(image, cv::Point(pred.box[0], pred.box[1]), cv::Point(pred.box[0]+pred.box[2],pred.box[1]+pred.box[3]), cv::Scalar(255,255,255), 2);
    cv::putText(image,class_names_[pred.label-1].c_str(), cv::Point(pred.box[0], pred.box[1]-5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,255), 1);
}