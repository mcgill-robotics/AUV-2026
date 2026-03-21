#pragma once

#include <array>
#include <string>
#include <opencv2/core.hpp>

// Object class enumeration
enum ObjectClass 
{
    BOARD_IMAGE,
    GATE_PREQUAL,
    SHARK,
    SAWFISH,
    MARKER_PREQUAL,
    WHITE_PIPE_MEDN
};

// Detection struct
struct DetectedObject 
{
    ObjectClass class_id;
    float confidence;
    cv::Rect bbox; // x, y, width, height
};

// Labels for object classes
inline const std::array<std::string, 6> ID_TO_LABEL = {
    "board_image",
    "gate_prequal",
    "shark",
    "sawfish",
    "marker_prequal",
    "white_pipe_medn"
};
