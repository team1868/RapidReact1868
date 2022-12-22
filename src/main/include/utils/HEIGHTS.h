#pragma once

// lower limit: 38497.0
// upper limit: 600500.0

constexpr double CLIMBER_TOLERANCE     = 500.0;
constexpr double PIVOT_ARM_TOLERANCE   = 4000.0;
constexpr double TELESCOPE_UPPER_LIMIT = 530000;  // 552500; // 530000; // 620000.0
constexpr double PIVOT_ARM_UPPER_LIMIT = 120000.0;

/**
 * [Up Toggle]
 * Schedules {RaiseTeleArmCommand}:
 * - Raises telescoping arm to just above R1
 */
constexpr double UP_SEQ_1_HEIGHT = 530000;  // 552500; // 575000; // 530000; //620000;
constexpr double UP_SEQ_TIMEOUT  = 3.0;

// Pull Up until limit switch
constexpr double PULL_UP_SEQ_1_HEIGHT = -100000.0;
// raise a bit more after pivot hooks engaged to get off bar
constexpr double PULL_UP_SEQ_2_HEIGHT = 185000.0;  // 300000.0;

// Reach Back
constexpr double REACH_BACK_SEQ_1_LENGTH      = 120000.0;  // Unravel the pivot arm (bring it out)
constexpr double REACH_BACK_SEQ_1_TELE_LENGTH = 300000.0;

// Raise To Next Rung
// Pulls up until limit switch
constexpr double RAISE_TO_RUNG_SEQ_1_HEIGHT = 530000;  // 552500; // 575000; // 530000; // 620000.0;

// Collapse
constexpr double COLLAPSE_SEQ_1_LENGTH = 65000.0;  // 45000.0; // 65000; // Bring in the pivot arm

// End Pull Up
// Pulls up slightly. Don’t want to end on static arms… just pull up a bit more
constexpr double END_PULL_UP_SEQ_1_HEIGHT = 260000.0;
