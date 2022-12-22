#include "microsystems/ControlBoard.h"

ControlBoard::ControlBoard() {}

double ControlBoard::GetLeftJoyX() { return _leftJoy.GetX(); }
double ControlBoard::GetLeftJoyY() { return _leftJoy.GetY(); }
double ControlBoard::GetRightJoyX() { return _rightJoy.GetX(); }
double ControlBoard::GetRightJoyY() { return _rightJoy.GetY(); }

double ControlBoard::GetDesired(Buttons button)
{
    switch (button) {
        case (kOuttakeLeftButton): return _leftJoy.GetRawButton(INTAKE_BUTTON_PORT);
        case (kIntakeRightButton): return _rightJoy.GetRawButton(INTAKE_BUTTON_PORT);
        case (kBlockerButton): return _operatorJoy.GetRawButton(BLOCKER_BUTTON_PORT);
        case (kShootButton): return _operatorJoy.GetRawButton(SHOOT_BUTTON_PORT);
        case (kFarShootPrepButton): return _operatorJoyB.GetRawButton(FAR_SHOT_PREP_BUTTON_PORT);
        case (kCloseShootPrepButton):
            return _operatorJoyB.GetRawButton(CLOSE_SHOT_PREP_BUTTON_PORT);
        case (kPivotArmOutButton): return _operatorJoyB.GetRawButton(PIVOT_OUT_BUTTON_PORT);
        case (kPivotArmInButton): return _operatorJoyB.GetRawButton(PIVOT_IN_BUTTON_PORT);
        case (kTeleArmUpButton): return _operatorJoy.GetRawButton(TELE_UP_BUTTON_PORT);
        case (kTeleArmDownButton): return _operatorJoy.GetRawButton(TELE_DOWN_BUTTON_PORT);
        case (kDefenseButton): return _operatorJoy.GetRawButton(DEFENSE_BUTTON_PORT);
        case (kAbortClimbButton): return _operatorJoyB.GetRawButton(ABORT_CLIMB_BUTTON_PORT);
        case (kUpButton): return _operatorJoyB.GetRawButton(CLIMB_UP_BUTTON_PORT);
        case (kTraversalButton): return _operatorJoyB.GetRawButton(TRAVERSAL_CLIMB_BUTTON_PORT);
        case (kHighClimbButton): return _operatorJoyB.GetRawButton(HIGH_CLIMB_BUTTON_PORT);
        case (kZeroTurretButton): return _operatorJoy.GetRawButton(ZERO_TURRET_BUTTON_PORT);
        case (kClimbTurretButton): return _operatorJoy.GetRawButton(CLIMB_TURRET_BUTTON_PORT);
        case (kGearShiftButton): return _rightJoy.GetRawButton(GEAR_SHIFT_BUTTON_PORT);
        case (kVisionPivotButtonLeft): return _leftJoy.GetRawButton(VISION_PIVOT_BUTTON_PORT);
        case (kVisionPivotButtonRight): return _rightJoy.GetRawButton(VISION_PIVOT_BUTTON_PORT);
        case (kResetTurretToDefaultButton):
            return _operatorJoy.GetRawButton(CLIMB_TURRET_BUTTON_PORT);
        case (kGroundOuttakeButton): return _leftJoy.GetRawButton(GROUND_OUTTAKE_BUTTON_PORT);
        default: return 0.0;
    }
}
