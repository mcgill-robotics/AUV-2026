#include <iostream>
#include <memory.h>
#include <format>

#include <object_tracker.hpp>

ObjectTracker::ObjectTracker(const float min_new_track_distance = 0.5) {
    this->min_new_track_distance = min_new_track_distance;
}

// destructor implicitly defined

void create_kf() {

}   // TODO: Add Eigen 3D vector type to constructor

void update() {

}   // TODO: Parameterize properly