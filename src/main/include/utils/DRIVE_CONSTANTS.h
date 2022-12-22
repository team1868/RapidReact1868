#pragma once

#include <units/acceleration.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>

#include "utils/CONSTANTS.h"
#include "utils/ROBOT_CONFIG.h"

namespace DriveConstants {
// 0.77701_V from sysID 2/21
constexpr auto ks = 0.72998_V;
// 2.3568 and 2.3568  from sysID
constexpr auto kv = 2.32_V / 1_mps;
// 0.42631 and 0.42631 from sysID
constexpr auto ka = 0.28885_V / 1_mps_sq;
// 2.3987; ramsete off: 0.6 to 0.65 tested on practice bot! 1.0 3.4472 0.53342 from sysID
constexpr double kPDriveVel = 0.6;
}  // namespace DriveConstants

/* ============= Robot Specific Properties ============= */
#ifdef JO
// 2.083333 ft measured 25 inches
constexpr units::meter_t DRIVETRAIN_TRACK_WIDTH = 25_in;
constexpr units::inch_t WHEEL_DIAMETER          = 6_in;

#elif defined(PRACTICEBOT) || defined(COMPBOT)

constexpr units::meter_t DRIVETRAIN_TRACK_WIDTH = 25_in;
constexpr units::inch_t WHEEL_DIAMETER          = 4_in;  // measurement in ft, 4 inch colsons for drivetrain
#endif

constexpr units::meter_t WHEEL_CIRCUMFERENCE  = WHEEL_DIAMETER * wpi::numbers::pi;
constexpr double NO_REDUCTION_TICKS_PER_METER = FALCON_ENCODER_TICKS / WHEEL_CIRCUMFERENCE.value();

constexpr double HIGH_GEAR_RATIO = 6.80;   // 58*34*34/(14*22*32.0)
constexpr double LOW_GEAR_RATIO  = 12.81;  // 58*34*44/(14*22*22.0)

// constexpr double HGEAR_TICKS_PER_METER = 41200.0;
constexpr double HGEAR_TICKS_PER_METER = HIGH_GEAR_RATIO * NO_REDUCTION_TICKS_PER_METER;
constexpr double LGEAR_TICKS_PER_METER = LOW_GEAR_RATIO * NO_REDUCTION_TICKS_PER_METER;

constexpr double DEADBAND_MAX            = 0.1;
constexpr double MAX_CURRENT_OUTPUT      = 180.0;  // Amps
constexpr double MAX_DRIVE_MOTOR_CURRENT = 40.0;   // Amps
constexpr double MIN_RATIO_ALL_CURRENT   = 0.2;
constexpr double MIN_RATIO_DRIVE_CURRENT = 0.3;
constexpr double MAX_RATIO_DRIVE_CURRENT = 0.7;
constexpr double MIN_BROWNOUT_VOLTAGE    = 7.5;
// constexpr double MAX_DRIVE_CURRENT_PERCENT = 1.0; // per motor, most teams are 40-50 Amps
