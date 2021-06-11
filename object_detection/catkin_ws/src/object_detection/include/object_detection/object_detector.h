#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>

struct Prediction{
    std::vector<float> box;
    float score;
    int label;
};

class ObjectDetector{
    public:
        ObjectDetector(const std::string &classes_filename, const std::string &model_filename,  const std::string &model_config, float score_threshold);
        int GetHeight(){return image_height_;}
        int GetWidth(){return image_width_;}
    
    protected:
        std::vector<Prediction> ProcessFrame(cv::Mat &image);

    private:
        void DrawBoxes(cv::Mat &image, Prediction pred);
        std::vector<std::string> class_names_;
        cv::dnn::Net model_;
        int image_height_ = 0;
        int image_width_ = 0;
        float score_threshold_ = 0.0;
};