
#include "zed_detection.hpp"

#include <exception>
#include <string>

// --- CUSTOM EXCEPTIONS (UNUSED) ---
// const char* ZedInitException::what() const noexcept
// {
// 	return error_details.c_str();
// }

// const char* YOLOLoadingException::what() const noexcept
// {
// 	return error_details.c_str();
// }

ZEDDetection::ZEDDetection(
		const string & yolo_model_path,
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
	): 
		frame_rate(frame_rate),
		confidence_threshold(confidence_threshold),
		max_range(max_range),
		min_new_track_distance(min_new_track_distance),
		use_stream(use_stream),
		stream_ip(stream_ip),
		stream_port(stream_port),
		show_detections(show_detections),
		debug_logs(debug_logs),
		log_error(log_error),
		log_info(log_info),
		log_warn(log_warn),
		log_warn_throttle(log_warn_throttle)
{
	if (!init_zed()) {
		log_error("Failed to initialize ZED camera");
		throw runtime_error("ZED initialization failed");
	}
    ZEDDetection::load_yolo_model(yolo_model_path);
}

bool ZEDDetection::init_zed()
{
	sl::InitParameters init_params;
	init_params.camera_resolution = sl::RESOLUTION::SVGA;
	init_params.camera_fps = frame_rate;
	init_params.coordinate_units = sl::UNIT::METER;
	init_params.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
	init_params.depth_mode = sl::DEPTH_MODE::NEURAL;
	init_params.depth_maximum_distance = max_range;
	init_params.enable_image_validity_check = true;

	if (use_stream) {
	    log_info("Connecting to stream: " + stream_ip + ":" + to_string(stream_port));
	    init_params.input.setFromStream(stream_ip.c_str(), stream_port);
	}

	sl::ERROR_CODE err = zed.open(init_params);
	if (err != sl::ERROR_CODE::SUCCESS) {
		log_error("Failed to open ZED: " + string(sl::toString(err).c_str()));
		return false;
	}

	// Enable positional tracking for VIO
	sl::PositionalTrackingParameters pos_param;
	pos_param.enable_imu_fusion = true;
	pos_param.set_floor_as_origin = false;
	err = zed.enablePositionalTracking(pos_param);
	if (err != sl::ERROR_CODE::SUCCESS) {
		log_error("Failed to enable positional tracking: " + string(sl::toString(err).c_str()));
		return false;
	}

	// Enable object detection (custom box mode - no ZED tracking)
	sl::ObjectDetectionParameters obj_param;
	obj_param.detection_model = sl::OBJECT_DETECTION_MODEL::CUSTOM_BOX_OBJECTS;
	obj_param.enable_tracking = false;
	obj_param.max_range = max_range;
	err = zed.enableObjectDetection(obj_param);
	if (err != sl::ERROR_CODE::SUCCESS) {
		log_error("Failed to enable object detection: " + string(sl::toString(err).c_str()));
		return false;
	}
	return true;
}

void ZEDDetection::load_yolo_model(const string& model_path)
{
	// Initialize YOLO
	log_info("Loading YOLO model from " + model_path);
    try {
        yolo_net = cv::dnn::readNetFromONNX(model_path);
        yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
        yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
		log_info("YOLO model loaded successfully");
    } catch (const cv::Exception& e) {
		log_error("Failed to load YOLO model: " + string(e.what()));
    }
}

void ZEDDetection::process_frame(double delta_time)
{
	if (delta_time <= 0) {
	    return; // Prevent division by zero on startup
	}

	if (!ZEDDetection::check_zed_status()) {
	    return; // Skip processing if ZED health not OK
	}

	cv::Mat img_bgr = get_cv_frame();

	
	// Run YOLO detection
	vector<DetectedObject> detections = run_yolo(img_bgr);
	if (detections.empty()) {
		log_warn_throttle("No detections from YOLO", 2000);
	    return; // No detections
	}
	
	vector<sl::CustomBoxObjectData> custom_boxes = detections_to_zed_2D_boxes(detections, img_bgr);
	
	// Ingest custom boxes
	zed.ingestCustomBoxObjects(custom_boxes);

	sl::Objects objects;
	zed.retrieveObjects(objects, obj_runtime_param);
	if (debug_logs) {
	    LogDebugTable(custom_boxes, objects);
	}
	// Get VIO pose
	sl::Pose cam_pose;
	sl::POSITIONAL_TRACKING_STATE tracking_state = zed.getPosition(cam_pose, sl::REFERENCE_FRAME::WORLD);
	// Check tracking state and warn every second
	if (tracking_state != sl::POSITIONAL_TRACKING_STATE::OK) {
	    log_warn_throttle("Error in VIO tracking", 1000);
	    return;
	}

}

bool ZEDDetection::check_zed_status()
{
    // Grab frame
	sl::ERROR_CODE grab_status = zed.grab(runtime_params);
	if (grab_status == sl::ERROR_CODE::CORRUPTED_FRAME) {
	    log_warn("Corrupted frame detected - skipping");
	    return false;
	}

	// only log failure to grab every second
	if (grab_status != sl::ERROR_CODE::SUCCESS) {
	    log_warn_throttle("Failed to grab frame", 1000);
	    return false;
	}

	sl::HealthStatus health = zed.getHealthStatus();
	// Check frame health, only log every 5 seconds
	if (health.low_image_quality) {
	    log_warn_throttle("Low image quality", 5000);
	    return false;
	}

	if (health.low_lighting) {
	    log_warn_throttle("Low light condition", 5000);
	    return false;
	}
	
	return true;
}

cv::Mat ZEDDetection::get_cv_frame()
{
	sl::Mat image;
    // Retrieve image
    zed.retrieveImage(image, sl::VIEW::LEFT);
    
    // Convert to OpenCV Mat
    return sl_mat_to_cv_mat(image);
}

vector<sl::CustomBoxObjectData> ZEDDetection::detections_to_zed_2D_boxes(const vector<DetectedObject>& detections, const cv::Mat& img_bgr)
{
	vector<sl::CustomBoxObjectData> custom_boxes;
	for (const auto& det : detections) {
	    sl::CustomBoxObjectData box;
	    box.unique_object_id = sl::generate_unique_id();
	    box.probability = det.confidence;
	    box.label = static_cast<int>(det.class_id);
	    box.is_grounded = false;
	    
	    // Convert cv::Rect to ZED bounding box
	    std::vector<sl::uint2> bbox_2d(4);
	    bbox_2d[0] = sl::uint2(det.bbox.x, det.bbox.y);
	    bbox_2d[1] = sl::uint2(det.bbox.x + det.bbox.width, det.bbox.y);
	    bbox_2d[2] = sl::uint2(det.bbox.x + det.bbox.width, det.bbox.y + det.bbox.height);
	    bbox_2d[3] = sl::uint2(det.bbox.x, det.bbox.y + det.bbox.height);
	    box.bounding_box_2d = bbox_2d;
	    
	    custom_boxes.push_back(box);

	    // Visualize detections
	    if (show_detections) {
		cv::rectangle(img_bgr, det.bbox, cv::Scalar(0, 255, 0), 2);
		string label = ID_TO_LABEL[det.class_id] + ": " + to_string(det.confidence).substr(0, 4);
		cv::putText(img_bgr, label, cv::Point(det.bbox.x, det.bbox.y - 5),
			    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
	    }
	}

	if (show_detections) {
	    cv::imshow("YOLO Detections", img_bgr);
	    cv::waitKey(1);
	}
}

vector<DetectedObject> ZEDDetection::run_yolo(const cv::Mat& img)
{
	vector<DetectedObject> detections;
	
	// Prepare input blob
	cv::Mat blob = cv::dnn::blobFromImage(img, 1.0/255.0, cv::Size(640, 640), cv::Scalar(), true, false);
	yolo_net.setInput(blob);
	
	// Forward pass
	vector<cv::Mat> outputs;
	yolo_net.forward(outputs, yolo_net.getUnconnectedOutLayersNames());
	
	// Parse YOLO output (YOLOv11 format)
	// Output shape: [1, 84, 8400] for 10 classes
	// [x, y, w, h, class_0_conf, class_1_conf, ..., class_9_conf]
	if (outputs.empty()) return detections;
	
	cv::Mat output = outputs[0];
	output = output.reshape(1, output.size[1]); // Reshape to [num_predictions, 84]
	cv::transpose(output, output); // Transpose to [8400, 84]
	
	float x_factor = img.cols / 640.0f;
	float y_factor = img.rows / 640.0f;
	
	for (int i = 0; i < output.rows; i++) {
	    cv::Mat row = output.row(i);
	    cv::Mat class_scores = row.colRange(4, 14); // 10 classes
	    
	    cv::Point class_id_point;
	    double max_confidence;
	    cv::minMaxLoc(class_scores, nullptr, &max_confidence, nullptr, &class_id_point);
	    
	    if (max_confidence > confidence_threshold) {
		float cx = row.at<float>(0) * x_factor;
		float cy = row.at<float>(1) * y_factor;
		float w = row.at<float>(2) * x_factor;
		float h = row.at<float>(3) * y_factor;
		
		int x = static_cast<int>(cx - w / 2);
		int y = static_cast<int>(cy - h / 2);
		
		DetectedObject det;
		det.class_id = static_cast<ObjectClass>(class_id_point.x);
		det.confidence = static_cast<float>(max_confidence);
		det.bbox = cv::Rect(x, y, static_cast<int>(w), static_cast<int>(h));
		
		detections.push_back(det);
	    }
	}
	return detections;
    }

std::tuple<vector<cv::Mat>, vector<cv::Mat>, vector<string>> ZEDDetection::determine_world_position_zed_2D_boxes(const sl::Objects& zed_2D_boxes,const sl::Pose& cam_pose)
    {
	// Retrieve 3D objects
	sl::Objects objects;
	zed.retrieveObjects(objects, obj_runtime_param);

	// Process measurements for tracking
	vector<cv::Mat> measurements;
	vector<cv::Mat> covariances;
	vector<string> classes;
	log_error("World Position Determination Not Implemented");
	return std::make_tuple(measurements, covariances, classes);
}

void ZEDDetection::LogDebugTable(const vector<sl::CustomBoxObjectData>& YOLO_detections, const sl::Objects& zed_detections)
{
    // Implementation of the debug table logging
}

ZEDDetection::~ZEDDetection()
{
    if (zed.isOpened()) {
        zed.close();
    }
}



cv::Mat sl_mat_to_cv_mat(sl::Mat& sl_image)
{
	int cv_type;
	switch (sl_image.getDataType()) {
	    case sl::MAT_TYPE::U8_C4:
		cv_type = CV_8UC4;
		break;
	    case sl::MAT_TYPE::U8_C3:
		cv_type = CV_8UC3;
		break;
	    default:
		cv_type = CV_8UC4;
	}
    cv::Mat cv_image(sl_image.getHeight(), sl_image.getWidth(), cv_type, sl_image.getPtr<sl::uchar1>());
	cv::Mat bgr_image;
	cv::cvtColor(cv_image, bgr_image, cv::COLOR_BGRA2BGR);
	return bgr_image;

}