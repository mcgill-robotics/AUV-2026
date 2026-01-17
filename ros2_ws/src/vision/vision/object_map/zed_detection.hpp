#pragma once

#include <vector>
#include <string>
#include <functional>
#include <tuple>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <sl/Camera.hpp>

#include "object.hpp"

using namespace std;
// --- CUSTOM EXCEPTIONS (UNUSED) ---
// class ZedInitException : public exception
// {
// public:
//     ZedInitException(const string& details) : error_details(string("Failed to initilize ZED camera: "+details)) {}
//     const char* what() const noexcept;
// private:
//     string error_details = "";
// };

// class YOLOLoadingException : public exception
// {
// public:
//     YOLOLoadingException(const string& details) : error_details(string("Failed to load YOLO model: "+details)) {}
//     const char* what() const noexcept;
// private:
//     string error_details = "";
// };

class ZEDDetection
{
public:
    ZEDDetection();
    ZEDDetection(
        const string & yolo_model_path,
        int yolo_input_size,
        int frame_rate,
        float confidence_threshold,
        float max_range,
        float min_new_track_distance,
        bool use_stream,
        const string & stream_ip,
        int stream_port,
        bool show_detections,
        bool debug_logs,
        function<void(const string&)> log_error,
        function<void(const string&)> log_info,
        function<void(const string&)> log_warn,
        function<void(const string&, int)> log_warn_throttle
    );
    void process_frame();
    void UpdateSensorDepth(double new_depth);
    std::tuple<vector<cv::Mat>, vector<cv::Mat>, vector<string>> GetData();
    
    ~ZEDDetection();
    
private:
    bool init_zed();
    
    void load_yolo_model(const string& model_path);

    bool check_zed_status();

    cv::Mat get_cv_frame();

    vector<sl::CustomBoxObjectData> detections_to_zed_2D_boxes(const vector<DetectedObject>& detections, const cv::Mat& img_bgr);

    vector<DetectedObject> run_yolo(const cv::Mat& img);
    cv::Mat letter_box(const cv::Mat& img, int target_size);

    cv::Mat transform_to_world(const sl::float3& local_pos, const sl::Rotation& rotation_matrix, const sl::float3& translation_vector);
    
    void determine_world_position_zed_2D_boxes(const sl::Objects&,const sl::Pose& cam_pose);

    
    void LogDebugTable(const vector<sl::CustomBoxObjectData>& YOLO_detections, const sl::Objects& zed_detections);


    int frame_rate;
    int YOLO_input_size;
    float confidence_threshold;
    float max_range;
    float min_new_track_distance;
    bool use_stream;
    string stream_ip;
    int stream_port;
    bool show_detections;
    bool debug_logs;
    
    double sensor_depth;
    double letter_box_scale;
    
    function<void(const string&)> log_error;
    function<void(const string&)> log_info;
    function<void(const string&)> log_warn;
    function<void(const string&, int)> log_warn_throttle;    
    cv::dnn::Net yolo_net;
    
    
    sl::Camera zed;
    sl::RuntimeParameters runtime_params;
    sl::ObjectDetectionRuntimeParameters obj_runtime_param;
    
    vector<cv::Mat> measurements;
	vector<cv::Mat> covariances;
	vector<string> classes;
};


cv::Mat sl_mat_to_cv_mat(sl::Mat& sl_image);

cv::Mat get_world_covariance(const float position_covariance[6], const sl::Rotation& rotation_matrix);
