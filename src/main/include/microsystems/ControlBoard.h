#pragma once

#include <frc/Joystick.h>
#include <frc2/command/button/JoystickButton.h>

#include "utils/BUTTON_PORTS.h"
#include "utils/PORTS.h"

class ControlBoard {
   private:
    // Joysticks for drive
    frc::Joystick _leftJoy{LEFT_JOY_USB_PORT}, _rightJoy{RIGHT_JOY_USB_PORT};
    // Joysticks for operator
    frc::Joystick _operatorJoy{OPERATOR_JOY_USB_PORT}, _operatorJoyB{OPERATOR_JOY_B_USB_PORT};

   public:
    ControlBoard();

    double GetLeftJoyX();
    double GetLeftJoyY();
    double GetRightJoyX();
    double GetRightJoyY();

    // Buttons on joysticks and operator controllers
    frc2::JoystickButton OuttakeLeftButton{&_leftJoy, INTAKE_BUTTON_PORT};
    frc2::JoystickButton IntakeRightButton{&_rightJoy, INTAKE_BUTTON_PORT};
    frc2::JoystickButton BlockerButton{&_operatorJoy, BLOCKER_BUTTON_PORT};
    frc2::JoystickButton ShootButton{&_operatorJoy, SHOOT_BUTTON_PORT};
    frc2::JoystickButton FarShootPrepButton{&_operatorJoyB, FAR_SHOT_PREP_BUTTON_PORT};
    frc2::JoystickButton CloseShootPrepButton{&_operatorJoyB, CLOSE_SHOT_PREP_BUTTON_PORT};
    frc2::JoystickButton PivotArmOutButton{&_operatorJoyB, PIVOT_OUT_BUTTON_PORT};
    frc2::JoystickButton PivotArmInButton{&_operatorJoyB, PIVOT_IN_BUTTON_PORT};
    frc2::JoystickButton TeleArmUpButton{&_operatorJoy, TELE_UP_BUTTON_PORT};
    frc2::JoystickButton TeleArmDownButton{&_operatorJoy, TELE_DOWN_BUTTON_PORT};
    frc2::JoystickButton DefenseButton{&_operatorJoy, DEFENSE_BUTTON_PORT};
    frc2::JoystickButton AbortClimbButton{&_operatorJoyB, ABORT_CLIMB_BUTTON_PORT};
    frc2::JoystickButton HighClimbButton{&_operatorJoyB, HIGH_CLIMB_BUTTON_PORT};
    frc2::JoystickButton ZeroTurretButton{&_operatorJoy, ZERO_TURRET_BUTTON_PORT};
    frc2::JoystickButton ClimbTurretButton{&_operatorJoy, CLIMB_TURRET_BUTTON_PORT};
    frc2::JoystickButton GearShiftButton{&_rightJoy, GEAR_SHIFT_BUTTON_PORT};
    frc2::JoystickButton VisionPivotButtonLeft{&_leftJoy, VISION_PIVOT_BUTTON_PORT};
    frc2::JoystickButton VisionPivotButtonRight{&_rightJoy, VISION_PIVOT_BUTTON_PORT};
    frc2::JoystickButton ResetTurretToDefaultButton{&_operatorJoy, CLIMB_TURRET_BUTTON_PORT};
    frc2::JoystickButton GroundOuttakeButton{&_leftJoy, GROUND_OUTTAKE_BUTTON_PORT};

    // kUpButton and kTraversalButton are the two buttons for a three-way switch
    frc2::JoystickButton TraversalButton{&_operatorJoyB, TRAVERSAL_CLIMB_BUTTON_PORT};
    frc2::JoystickButton UpButton{&_operatorJoyB, CLIMB_UP_BUTTON_PORT};

    double GetDesired(Buttons button);
};
