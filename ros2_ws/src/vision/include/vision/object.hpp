#pragma once

#include <array>
#include <string>
#include <opencv2/core.hpp>

// Object class enumeration
enum ObjectClass 
{
    GATE, 
    LANE_MARKER, 
    RED_PIPE, 
    WHITE_PIPE, 
    OCTAGON, 
    TABLE, 
    BIN, 
    BOARD, 
    SHARK, 
    SAWFISH
};

// Detection struct
struct DetectedObject 
{
    ObjectClass class_id;
    float confidence;
    cv::Rect bbox; // x, y, width, height
};

// Labels for object classes
inline const std::array<std::string, 10> ID_TO_LABEL = {
	"gate",
	"lane_marker",
	"red_pipe",
	"white_pipe",
	"octagon",
	"table",
	"bin",
	"board",
	"shark",
	"sawfish"
};
