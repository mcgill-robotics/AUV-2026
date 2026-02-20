#include "sensors/depth_calibration.hpp"
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace sensors
{
const static std::string share_path = ament_index_cpp::get_package_share_directory("sensors");
const static std::string yaml_path = share_path + "/config/depth_configuration.yaml";

static double depth_slope;
static double depth_offset;

//Convert from sensor values to expected depth values
double get_calibrated_depth(double base_depth) {
    return base_depth * depth_slope + depth_offset;
}

void load_depth_configuration() {
    YAML::Node depth_config = YAML::LoadFile(yaml_path);

    double depth_min_actual = depth_config["depth_min"]["actual"].as<double>();
    double depth_min_sensor = depth_config["depth_min"]["sensor"].as<double>();
    double depth_max_actual = depth_config["depth_max"]["actual"].as<double>();
    double depth_max_sensor = depth_config["depth_max"]["sensor"].as<double>();

    double diff_expected = depth_max_actual - depth_min_actual;
    double diff_sensor = depth_max_sensor - depth_min_sensor;

    depth_slope = diff_expected / diff_sensor;

    depth_offset = depth_min_actual - depth_min_sensor;
}
}