#include "commands/climbcommands/ManualPivotArmCommand.h"

#include "microsystems/ControlBoard.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"
#include "utils/CLIMBER_CONSTANTS.h"
#include "utils/HEIGHTS.h"

ManualPivotArmCommand::ManualPivotArmCommand(Climber& climber,
                                             SensorBoard& sensorControl,
                                             ControlBoard& humanControl)
    : _climber{climber}, _sensorControl{sensorControl}, _humanControl{humanControl}
{
    AddRequirements({&climber});
}

void ManualPivotArmCommand::Initialize()
{
    _turretGood = false;
    _sensorControl.SetDesiredColor(7);
}

void ManualPivotArmCommand::Execute()
{
#ifdef COMPBOT
    double pivotArmOutput = RESET_OUTPUT;
    _turretGood           = _turretGood || _sensorControl.IsAtTurretClimbAngle();
    if (_turretGood) {
        if (_humanControl.GetDesired(Buttons::kPivotArmOutButton)) {
            if (_climber.GetResetClimbEncoders()) {
                pivotArmOutput = 0.5;
            } else if (_climber.GetPivotArmEncoderValue() < PIVOT_ARM_UPPER_LIMIT) {
                pivotArmOutput = PIVOT_ARM_OUT_OUTPUT;
            }
        } else if (_humanControl.GetDesired(Buttons::kPivotArmInButton) &&
                   _climber.GetHardStopLimitSwitch()) {
            pivotArmOutput = _climber.GetResetClimbEncoders() ? -0.5 : PIVOT_ARM_IN_OUTPUT;
        }
    }

    _climber.SetPivotArmOutput(pivotArmOutput);
#endif
}

void ManualPivotArmCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _climber.SetPivotArmOutput(RESET_OUTPUT);
}

bool ManualPivotArmCommand::IsFinished()
{
    return !_humanControl.GetDesired(Buttons::kPivotArmInButton) &&
           !_humanControl.GetDesired(Buttons::kPivotArmOutButton);
}