#pragma once

// Buttons mapping
constexpr int INTAKE_BUTTON_PORT          = 1;
constexpr int GROUND_OUTTAKE_BUTTON_PORT  = 2;
constexpr int BLOCKER_BUTTON_PORT         = 9;
constexpr int SHOOT_BUTTON_PORT           = 12;  // check
constexpr int FAR_SHOT_PREP_BUTTON_PORT   = 6;
constexpr int CLOSE_SHOT_PREP_BUTTON_PORT = 7;
constexpr int PIVOT_OUT_BUTTON_PORT       = 9;
constexpr int PIVOT_IN_BUTTON_PORT        = 10;
constexpr int TELE_UP_BUTTON_PORT         = 5;
constexpr int TELE_DOWN_BUTTON_PORT       = 4;
constexpr int DEFENSE_BUTTON_PORT         = 3;
constexpr int ABORT_CLIMB_BUTTON_PORT     = 8;
constexpr int CLIMB_UP_BUTTON_PORT        = 5;
constexpr int TRAVERSAL_CLIMB_BUTTON_PORT = 12;
constexpr int HIGH_CLIMB_BUTTON_PORT      = 11;

constexpr int ZERO_TURRET_BUTTON_PORT  = 7;  // A
constexpr int CLIMB_TURRET_BUTTON_PORT = 8;  //
constexpr int GEAR_SHIFT_BUTTON_PORT   = 3;  // TODO: is this left or right?
constexpr int VISION_PIVOT_BUTTON_PORT = 2;

enum Buttons {
    kOuttakeLeftButton,
    kIntakeRightButton,
    kBlockerButton,
    kShootButton,
    kFarShootPrepButton,
    kCloseShootPrepButton,
    kPivotArmOutButton,
    kPivotArmInButton,
    kTeleArmUpButton,
    kTeleArmDownButton,
    kDefenseButton,
    kAbortClimbButton,
    kUpButton,
    kTraversalButton,
    kHighClimbButton,
    kZeroTurretButton,
    kClimbTurretButton,
    kGearShiftButton,
    kVisionPivotButtonLeft,
    kVisionPivotButtonRight,
    kResetTurretToDefaultButton,
    kGroundOuttakeButton
};
