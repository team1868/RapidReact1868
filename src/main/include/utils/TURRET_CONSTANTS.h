#pragma once

constexpr double POTENTIOMETER_MAX = 3400.0;
constexpr double POTENTIOMETER_MIN = 200.0;

constexpr double ANGLE_TO_POT = 10.0;
constexpr double POT_TO_ANGLE = 1.0 / ANGLE_TO_POT;
constexpr double ANGLE_TO_ENC = 159.3;
constexpr double ENC_TO_ANGLE = 1.0 / ANGLE_TO_ENC;

constexpr double POT_TO_ENC = ANGLE_TO_ENC / ANGLE_TO_POT;
constexpr double ENC_TO_POT = ANGLE_TO_POT / ANGLE_TO_ENC;

constexpr double ENC_TOLERANCE = ANGLE_TO_ENC / 2.0;
constexpr double R_TOLERANCE   = ANGLE_TO_POT / 2.0;

constexpr double POTENTIOMETER_OFFSET = 2780.0;
constexpr double TURRET_MAX_OUTPUT    = 0.4;

constexpr double FULL_TURRET_RANGE = 3600.0;
constexpr double ENCODER_MIN       = 0;
constexpr double ENCODER_MAX       = (POTENTIOMETER_MAX - POTENTIOMETER_MIN) * POT_TO_ENC;

constexpr double CLIMB_TURRET_ANGLE = 1880.0;
constexpr double ZERO_TURRET_ANGLE  = 2780.0;
