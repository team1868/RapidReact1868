#include "commands/climbcommands/CollapseCommand.h"

#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"
#include "utils/HEIGHTS.h"

CollapseCommand::CollapseCommand(Climber& climber, SensorBoard& sensorControl)
    : _climber{climber}, _sensorControl{sensorControl}
{
    AddRequirements({&climber});
}

void CollapseCommand::Initialize()
{
    _turretGood = false;
    _sensorControl.SetAutoClimbing(true);
    _sensorControl.SetDesiredColor(4);
}

void CollapseCommand::Execute()
{
    _turretGood = _turretGood || _sensorControl.IsAtTurretClimbAngle();
    if (!_turretGood) {
        _climber.SetPivotArmTargetPosition(_climber.GetPivotArmEncoderValue());
        return;
    }
    _sensorControl.SetCurrentlyClimbing(true);

    if (!_climber.GetHardStopLimitSwitch()) {
        // switch seen
        _climber.SetPivotArmOutput(RESET_OUTPUT);
    } else {
        _climber.SetPivotArmTargetPosition(COLLAPSE_SEQ_1_LENGTH);  // length JUST to collapse
    }
}

void CollapseCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _sensorControl.SetCurrentlyClimbing(false);
    _climber.SetPivotArmOutput(RESET_OUTPUT);
    _climber.SetTelescopeArmOutput(RESET_OUTPUT);
    _sensorControl.SetAutoClimbing(false);
}

bool CollapseCommand::IsFinished()
{
    return _turretGood &&
           fabs(_climber.GetPivotArmEncoderValue() - COLLAPSE_SEQ_1_LENGTH) < PIVOT_ARM_TOLERANCE;
}