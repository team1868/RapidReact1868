#pragma once

#include "utils/ROBOT_CONFIG.h"

#ifdef COMPBOT
constexpr double INTAKE_ROLLERS_OUTPUT      = 1.0;
constexpr double INTAKE_UNDO_ROLLERS_OUTPUT = -0.75;
constexpr double REV_INTAKE_ROLLERS_OUTPUT  = -1.0;

#else

constexpr double INTAKE_ROLLERS_OUTPUT      = 1.0;
constexpr double INTAKE_UNDO_ROLLERS_OUTPUT = -0.75;
constexpr double REV_INTAKE_ROLLERS_OUTPUT  = -1.0;
#endif
