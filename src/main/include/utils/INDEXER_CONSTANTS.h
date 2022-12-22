#pragma once

#include "utils/ROBOT_CONFIG.h"

#ifdef COMPBOT
constexpr double ELEVATOR_OUTPUT           = 0.5;
constexpr double REV_ELEVATOR_OUTPUT       = -0.3;
constexpr double INDEX_ROLLERS_OUTPUT      = 0.5;
constexpr double REV_INDEX_ROLLERS_OUTPUT  = -0.7;
constexpr double INDEX_UNDO_ROLLERS_OUTPUT = -0.75;

#else

constexpr double ELEVATOR_OUTPUT           = 0.5;
constexpr double REV_ELEVATOR_OUTPUT       = -0.5;
constexpr double INDEX_ROLLERS_OUTPUT      = 0.7;
constexpr double REV_INDEX_ROLLERS_OUTPUT  = -0.7;
constexpr double INDEX_UNDO_ROLLERS_OUTPUT = -0.75;
#endif
