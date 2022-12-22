#pragma once

#include <units/angle.h>
#include <units/length.h>

using namespace units;
using namespace units::angle;
using namespace units::length;

constexpr double ACTUAL_TARGET_HEIGHT = 104 / 12.0;
constexpr meter_t CAMERA_HEIGHT_M     = 1.2192_m;
constexpr meter_t TARGET_HEIGHT_M     = 2.6416_m;
constexpr degree_t CAMERA_PITCH_M     = 30_deg;
