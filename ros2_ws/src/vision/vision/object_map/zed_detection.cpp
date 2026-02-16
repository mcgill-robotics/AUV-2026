#include "zed_detection.hpp"
#include "object.hpp"
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
		int frame_rate,
		float confidence_threshold,
		float max_range,
		bool use_stream,
		const string & stream_ip,
		int stream_port,
		bool show_detections,
		bool debug_logs,
		function<void(const string&)> log_error,
		function<void(const string&)> log_fatal,
		function<void(const string&)> log_info,
        function<void(const string&)> log_warn,
		function<void(const string&, int)> log_warn_throttle
	): 
		frame_rate(frame_rate),
		confidence_threshold(confidence_threshold),
		max_range(max_range),
		use_stream(use_stream),
		stream_ip(stream_ip),
		stream_port(stream_port),
		show_detections(show_detections),
		debug_logs(debug_logs),
		log_error(log_error),
		log_fatal(log_fatal),
		log_info(log_info),
		log_warn(log_warn),
		log_warn_throttle(log_warn_throttle)
{
	if (!init_zed()) {
		log_error("Failed to initialize ZED camera");
		log_fatal("Exiting due to ZED initialization failure");
		throw runtime_error("ZED initialization failed");
	}
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

// REMOVED: load_yolo_model

void ZEDDetection::process_detections(const std::vector<sl::CustomBoxObjectData>& detections)
{
	if (!ZEDDetection::check_zed_status()) {
	    return; // Skip processing if ZED health not OK
	}


	
	// Ingest custom boxes
	zed.ingestCustomBoxObjects(detections);

	sl::Objects objects;
	zed.retrieveObjects(objects, obj_runtime_param);
	if (debug_logs) {
	    LogDebugTable(detections, objects);
	}
	// Get VIO pose
	sl::Pose cam_pose;
	sl::POSITIONAL_TRACKING_STATE tracking_state = zed.getPosition(cam_pose, sl::REFERENCE_FRAME::WORLD);
	// Check tracking state and warn every second
	if (tracking_state != sl::POSITIONAL_TRACKING_STATE::OK) {
	    log_warn_throttle("Error in VIO tracking", 1000);
	    return;
	}
	determine_world_position_zed_2D_boxes(objects, cam_pose);
}

std::tuple<vector<Eigen::Vector3d>, vector<Eigen::Matrix3d>, vector<string>, 
vector<double>, vector <double>> ZEDDetection::GetDetections()
{
	return {measurements, covariances, classes, orientations, confidences};
}

std::tuple<Eigen::Vector3d,Eigen::Vector4d> ZEDDetection::GetCameraPose()
{
	Eigen::Vector3d position(pose_translation.x, pose_translation.y, -sensor_depth);
	Eigen::Vector4d orientation(pose_orientation.w, pose_orientation.x, pose_orientation.y, pose_orientation.z);
	return {position, orientation};
}

ZEDDetection::~ZEDDetection()
{
    if (zed.isOpened()) {
        zed.close();
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
	    log_warn_throttle("Low light conditions", 5000);
	    return false;
	}
	
	return true;
}

// REMOVED: get_cv_frame

// REMOVED: run_yolo

// REMOVED: letter_box

void ZEDDetection::determine_world_position_zed_2D_boxes(const sl::Objects& zed_objects,const sl::Pose& cam_pose)
    {

	// Process measurements for tracking
	measurements.clear();
	covariances.clear();
	classes.clear();
	sl::Rotation rotation = cam_pose.getRotationMatrix();
	pose_translation = cam_pose.getTranslation();
	pose_orientation = cam_pose.getOrientation();
	for (const auto& obj : zed_objects.object_list) {
		sl::float3 pos = obj.position;
		
		// Check for invalid positions
		if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.z) ||
		std::isinf(pos.x) || std::isinf(pos.y) || std::isinf(pos.z) ||
		pos.x < 0) {
			continue;
		}

		// Get label
		int label_idx = obj.raw_label;
		string label_str = (static_cast<size_t>(label_idx) < ID_TO_LABEL.size()) ? string(ID_TO_LABEL[label_idx]) : to_string(label_idx);

		// Transform to world frame
		Eigen::Vector3d world_pos = transform_to_world(pos, rotation, pose_translation);

		// Get covariance in world frame
		Eigen::Matrix3d cov_world = get_world_covariance(obj.position_covariance, rotation);

		// Filter: Skip far-away pipes
		if (label_str == "red_pipe" || label_str == "white_pipe") {
			float dist = std::sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
			if (dist > 7.0) {
				continue;
			}
		}

		measurements.push_back(world_pos);
		covariances.push_back(cov_world);
		classes.push_back(label_str);
		// TODO: add orientation estimation, for now it's Axis Aligned so 0
		orientations.push_back(0.0);
		confidences.push_back(obj.confidence);
	}
}

void ZEDDetection::LogDebugTable(const vector<sl::CustomBoxObjectData>& YOLO_detections, const sl::Objects& zed_detections)
{
    // Implementation of the debug table logging
    (void)YOLO_detections;
    (void)zed_detections;
}

void ZEDDetection::UpdateSensorDepth(double new_depth)
{
	sensor_depth = new_depth;
}

Eigen::Vector3d ZEDDetection::transform_to_world(const sl::float3& local_pos, const sl::Rotation& rotation_matrix, const sl::float3& translation_vector)
{
	Eigen::Vector3d pos;
    pos(0) = local_pos.x;
    pos(1) = local_pos.y;
    pos(2) = local_pos.z;

    Eigen::Matrix3d R = Zed_Rotation_to_Eigen(rotation_matrix);
	
    Eigen::Vector3d t;
    t(0) = translation_vector.x;
    t(1) = translation_vector.y;
    t(2) = -sensor_depth; // Override Z with pressure sensor
	
    Eigen::Vector3d world_pos = t + R * pos;
    return world_pos;
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
Eigen::Matrix3d get_world_covariance(const float position_covariance[6], const sl::Rotation& rotation_matrix)
{
	Eigen::Matrix3d cov_cam;
	cov_cam(0, 0) = position_covariance[0];
	cov_cam(0, 1) = position_covariance[1];
	cov_cam(0, 2) = position_covariance[2];
	cov_cam(1, 0) = position_covariance[1];
	cov_cam(1, 1) = position_covariance[3];
	cov_cam(1, 2) = position_covariance[4];
	cov_cam(2, 0) = position_covariance[2];
	cov_cam(2, 1) = position_covariance[4];
	cov_cam(2, 2) = position_covariance[5];

	Eigen::Matrix3d R = Zed_Rotation_to_Eigen(rotation_matrix);

	Eigen::Matrix3d cov_world = R * cov_cam * R.transpose();
	return cov_world;
}

Eigen::Matrix3d Zed_Rotation_to_Eigen(const sl::Rotation& rotation_matrix)
{
    Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rowmajor =
        Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(rotation_matrix.r);
	Eigen::Matrix3d mat = rowmajor.cast<double>();
	
    return mat; // converts to column-major Eigen::Matrix3f
}