#pragma once

#include <frc/I2C.h>
#include <frc/PneumaticsModuleType.h>
#include <frc/PowerDistribution.h>

#include "utils/ROBOT_CONFIG.h"

// General
constexpr double RESET_OUTPUT = 0.0;

/* ============= Driver Station ============= */
constexpr int LEFT_JOY_USB_PORT       = 1;
constexpr int RIGHT_JOY_USB_PORT      = 0;
constexpr int OPERATOR_JOY_USB_PORT   = 2;
constexpr int OPERATOR_JOY_B_USB_PORT = 3;

/* ============= Robot Independent IDs ============= */
// Indexer
constexpr int INDEX_FUNNEL_MOTOR_A_ID           = 5;   // right side
constexpr int INDEX_FUNNEL_MOTOR_B_ID           = 11;  // left side
constexpr int ELEVATOR_FEEDER_MOTOR_ID          = 4;
constexpr int TOP_ELEVATOR_LIGHT_SENSOR_PORT    = 3;
constexpr int BOTTOM_ELEVATOR_LIGHT_SENSOR_PORT = 2;
constexpr int FUNNEL_LIGHT_SENSOR_PORT          = 4;

// LED
constexpr int LED_LENGTH = 38;
constexpr int LED_PORT   = 7;

// Turret -- IF ROBORIO I2C PORT DIES frc::I2C::Port::kOnboard;
constexpr auto I2CPORT            = frc::I2C::Port::kMXP;
constexpr int COLOR_SENSOR_NUMBER = 1;

/* ============= Robot Specific IDs ============= */
#ifdef JO
/* ============= Pneumatics ============= */
constexpr int PNEUMATICS_MODULE_ID                         = 0;
constexpr frc::PneumaticsModuleType PNEUMATICS_MODULE_TYPE = frc::PneumaticsModuleType::CTREPCM;
constexpr frc::PowerDistribution::ModuleType PDP_MODULE_TYPE =
    frc::PowerDistribution::ModuleType::kCTRE;
constexpr int PDP_ID = 0;

/* ============= PDP Channels ============= */
constexpr int LEFT_DRIVE_MOTOR_A_PDP_CHAN  = 0;
constexpr int LEFT_DRIVE_MOTOR_B_PDP_CHAN  = 1;
constexpr int RIGHT_DRIVE_MOTOR_A_PDP_CHAN = 15;
constexpr int RIGHT_DRIVE_MOTOR_B_PDP_CHAN = 14;

#elif defined(PRACTICEBOT) || defined(COMPBOT)

/* ============= Pneumatics ============= */
constexpr int PNEUMATICS_MODULE_ID                         = 1;
constexpr frc::PneumaticsModuleType PNEUMATICS_MODULE_TYPE = frc::PneumaticsModuleType::REVPH;
constexpr frc::PowerDistribution::ModuleType PDP_MODULE_TYPE =
    frc::PowerDistribution::ModuleType::kRev;
constexpr int PDP_ID = 1;

/* ============= PDP Channels ============= */
constexpr int LEFT_DRIVE_MOTOR_A_PDP_CHAN  = 0;
constexpr int LEFT_DRIVE_MOTOR_B_PDP_CHAN  = 1;
constexpr int RIGHT_DRIVE_MOTOR_A_PDP_CHAN = 15;
constexpr int RIGHT_DRIVE_MOTOR_B_PDP_CHAN = 14;
#endif

#ifdef COMPBOT
/* ============= Solenoid Ports ============= */
// Blocker
constexpr int BLOCKER_SOLENOID_PORT = 8;

// Climber
constexpr int CLIMBER_SOLENOID_FORWARD_PORT = 2;
constexpr int HOOK_SOLENOID_PORT            = 5;

// Drivetrain
constexpr int GEAR_SHIFT_SOLENOID_PORT = 6;
constexpr int LIGHT_SOLENOID_PORT      = 4;

// Intake
constexpr int INTAKE_SOLENOID_FORWARD_PORT = 7;

/* ============= Motor IDs and ports ============= */
// Climber
constexpr int TELESCOPE_ARM_MOTOR_ID = 8;

// Drivetrain
constexpr int RIGHT_DRIVE_MOTOR_ID   = 1;
constexpr int RIGHT_DRIVE_MOTOR_2_ID = 2;
constexpr int LEFT_DRIVE_MOTOR_ID    = 3;
constexpr int LEFT_DRIVE_MOTOR_2_ID  = 4;

// Indexer
constexpr int ELEVATOR_MOTOR_ID     = 11;
constexpr int INDEX_ROLLER_MOTOR_ID = 6;
constexpr int PIVOT_ARM_MOTOR_ID    = 7;

// Intake
constexpr int INTAKE_ROLLERS_MOTOR_ID = 9;

// Shooter
constexpr int FLYWHEEL_MOTOR_1_ID = 20;
constexpr int FLYWHEEL_MOTOR_2_ID = 21;
constexpr int HOOD_1_PORT         = 8;
constexpr int HOOD_2_PORT         = 9;

// Turret
constexpr int TURRET_WRIST_MOTOR_ID = 10;

/* ============= Sensors ============= */
// Climber
constexpr int LEFT_CLIMBER_LIMIT_SWITCH_PORT    = 0;
constexpr int RIGHT_CLIMBER_LIMIT_SWITCH_PORT   = 1;
constexpr int PIVOT_HARD_STOP_LIMIT_SWITCH_PORT = 4;
constexpr int TELE_HARD_STOP_LIMIT_SWITCH_PORT  = 5;

// Indexer
constexpr int TOP_LIGHT_SENSOR_PORT    = 2;
constexpr int BOTTOM_LIGHT_SENSOR_PORT = 3;

// Turret
constexpr int ANALOG_POTENTIOMETER_PORT = 3;
#endif

#ifdef JO
/* ============= Solenoid Ports ============= */
// Climber
constexpr int CLIMBER_SOLENOID_FORWARD_PORT = 10;
constexpr int RATCHET_SOLENOID_FORWARD_PORT = 2;
constexpr int CLIMBER_SOLENOID_REVERSE_PORT = 12;
constexpr int RATCHET_SOLENOID_REVERSE_PORT = 13;

// Drivetrain
constexpr int GEAR_SHIFT_SOLENOID_PORT = 1;
constexpr int LIGHT_SOLENOID_PORT      = 4;

// Intake
constexpr int INTAKE_SOLENOID_PORT         = 1;  // not used
constexpr int INTAKE_SOLENOID_FORWARD_PORT = 1;
constexpr int INTAKE_WRIST_MOTOR_ID        = 2;  // not used
constexpr int INTAKE_ROLLERS_MOTOR_ID      = 2;

/* ============= Motor IDs and ports ============= */
// Climber
constexpr int TELESCOPE_ARM_MOTOR_ID = 26;  // TODO FIX

// Drivetrain
constexpr int RIGHT_DRIVE_MOTOR_ID   = 0;
constexpr int RIGHT_DRIVE_MOTOR_2_ID = 1;
constexpr int LEFT_DRIVE_MOTOR_ID    = 2;
constexpr int LEFT_DRIVE_MOTOR_2_ID  = 3;

// Indexer
constexpr int ELEVATOR_MOTOR_ID     = 8;
constexpr int INDEX_ROLLER_MOTOR_ID = 4;

// Shooter
constexpr int FLYWHEEL_MOTOR_1_ID = 20;
constexpr int FLYWHEEL_MOTOR_2_ID = 21;

// Turret
constexpr int TURRET_WRIST_MOTOR_ID = 12;

/* ============= Sensors ============= */
// Climber
constexpr int LEFT_CLIMBER_LIMIT_SWITCH_PORT    = 1;
constexpr int RIGHT_CLIMBER_LIMIT_SWITCH_PORT   = 7;
constexpr int PIVOT_HARD_STOP_LIMIT_SWITCH_PORT = -1;
constexpr int TELE_HARD_STOP_LIMIT_SWITCH_PORT  = -1;

// Indexer
constexpr int TOP_LIGHT_SENSOR_PORT    = 3;
constexpr int BOTTOM_LIGHT_SENSOR_PORT = 4;

// Turret
constexpr int ANALOG_POTENTIOMETER_PORT = 0;  // correct for nova
#endif

#ifdef PRACTICEBOT
/* ============= Solenoid Ports ============= */
// Climber
constexpr int CLIMBER_SOLENOID_FORWARD_PORT = 2;

// Drivetrain
constexpr int GEAR_SHIFT_SOLENOID_PORT = 0;
constexpr int LIGHT_SOLENOID_PORT      = 4;

// Intake
constexpr int INTAKE_SOLENOID_FORWARD_PORT = 7;

/* ============= Motor IDs and ports ============= */
// Climber
constexpr int TELESCOPE_ARM_MOTOR_ID = 8;

// Drivetrain
constexpr int RIGHT_DRIVE_MOTOR_ID   = 1;
constexpr int RIGHT_DRIVE_MOTOR_2_ID = 2;
constexpr int LEFT_DRIVE_MOTOR_ID    = 3;
constexpr int LEFT_DRIVE_MOTOR_2_ID  = 4;

// Indexer
constexpr int ELEVATOR_MOTOR_ID     = 11;
constexpr int INDEX_ROLLER_MOTOR_ID = 6;

// Intake
constexpr int INTAKE_ROLLERS_MOTOR_ID = 9;

// Shooter
constexpr int FLYWHEEL_MOTOR_1_ID = 20;
constexpr int FLYWHEEL_MOTOR_2_ID = 21;
constexpr int HOOD_1_PORT         = 8;
constexpr int HOOD_2_PORT         = 9;

// Turret
constexpr int TURRET_WRIST_MOTOR_ID = 10;

/* ============= Sensors ============= */
// Climber
constexpr int LEFT_CLIMBER_LIMIT_SWITCH_PORT    = 0;
constexpr int RIGHT_CLIMBER_LIMIT_SWITCH_PORT   = 1;
constexpr int PIVOT_HARD_STOP_LIMIT_SWITCH_PORT = 4;
constexpr int TELE_HARD_STOP_LIMIT_SWITCH_PORT  = 5;

// Indexer
constexpr int TOP_LIGHT_SENSOR_PORT    = 2;
constexpr int BOTTOM_LIGHT_SENSOR_PORT = 3;

// Turret
constexpr int ANALOG_POTENTIOMETER_PORT = 3;
#endif
