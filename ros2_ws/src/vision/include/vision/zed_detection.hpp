#pragma once

#include <vector>
#include <string>
#include <functional>
#include <tuple>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <sl/Camera.hpp>
#include <Eigen/Dense>

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
    /// @brief Process a single frame from the ZED camera, performing object detection and storing results internally.
    void process_frame();
    /// @brief Update the sensor depth used for world position calculations.
    /// @param new_depth The new depth of the sensor in meters.
    void UpdateSensorDepth(double new_depth);
    /// @brief Retrieve the detected object data including positions, covariances, and class labels.
    /// @return A tuple containing vectors of positions, covariances, and class labels.
    std::tuple<vector<Eigen::Vector3d>, vector<Eigen::Matrix3d>, vector<string>,
    vector<double>, vector<double>> GetDetections();
    
    std::tuple<Eigen::Vector3d,Eigen::Vector4d> GetCameraPose();
    ~ZEDDetection();
    
private:
    bool init_zed();
    
    void load_yolo_model(const string& model_path);

    bool check_zed_status();
    // Capture a frame from the ZED camera and convert it to an OpenCV Mat.
    cv::Mat get_cv_frame();

    // Run YOLO object detection on the provided image.
    vector<sl::CustomBoxObjectData> run_yolo(const cv::Mat& img);
    // Resize and pad the image to fit the YOLO model's input size while maintaining aspect ratio.
    cv::Mat letter_box(const cv::Mat& img, int target_size);

    // Determine the world positions of detected objects using ZED SDK 2D boxes and camera pose.
    void determine_world_position_zed_2D_boxes(const sl::Objects&,const sl::Pose& cam_pose);

    // Transform local object positions to world coordinates using the camera's pose.
    Eigen::Vector3d transform_to_world(const sl::float3& local_pos, const sl::Rotation& rotation_matrix, const sl::float3& translation_vector);

    // TODO: implement debug table logging to compare YOLO and ZED detections
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
    
    vector<Eigen::Vector3d> measurements;
	vector<Eigen::Matrix3d> covariances;
    vector<double> orientations;
    vector<double> confidences;
	vector<string> classes;
	sl::float3 pose_translation;
	sl::Orientation pose_orientation;
};


cv::Mat sl_mat_to_cv_mat(sl::Mat& sl_image);

Eigen::Matrix3d get_world_covariance(const float position_covariance[6], const sl::Rotation& rotation_matrix);

Eigen::Matrix3d Zed_Rotation_to_Eigen(const sl::Rotation& rotation_matrix);