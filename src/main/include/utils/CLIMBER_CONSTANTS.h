#pragma once

#include "utils/ROBOT_CONFIG.h"

constexpr double ABORT_CLIMB_TIMEOUT = 0.5;
constexpr int TELESCOPE_PID_LOOP_ID  = 0;
constexpr int PIVOT_ARM_PID_LOOP_ID  = 0;

#ifdef COMPBOT
// Climber
constexpr double TELESCOPE_UP_OUTPUT   = 1.0;
constexpr double TELESCOPE_DOWN_OUTPUT = -1.0;
constexpr double PIVOT_ARM_IN_OUTPUT   = -0.5;
constexpr double PIVOT_ARM_OUT_OUTPUT  = 0.5;

#else

// Climber
constexpr double TELESCOPE_UP_OUTPUT   = 1.0;
constexpr double TELESCOPE_DOWN_OUTPUT = -1.0;
constexpr double PIVOT_ARM_IN_OUTPUT   = 1.0;
constexpr double PIVOT_ARM_OUT_OUTPUT  = -1.0;
#endif
