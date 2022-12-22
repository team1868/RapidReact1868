#include "commands/climbcommands/RaiseTeleArmCommand.h"

#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"
#include "utils/HEIGHTS.h"

RaiseTeleArmCommand::RaiseTeleArmCommand(Climber& climber, SensorBoard& sensorControl)
    : _climber{climber}, _sensorControl{sensorControl}
{
    AddRequirements({&climber});
}

void RaiseTeleArmCommand::Initialize()
{
    _startTime  = _sensorControl.GetCurTime();
    _turretGood = false;

    _sensorControl.SetAutoClimbing(true);
    _sensorControl.SetCurrentlyClimbing(true);
    _sensorControl.SetDesiredColor(7);
}

void RaiseTeleArmCommand::Execute()
{
    _turretGood = _turretGood || _sensorControl.IsAtTurretClimbAngle();
    if (!_turretGood) {
        _startTime = _sensorControl.GetCurTime();
        _climber.SetTargetPosition(_climber.GetEncoderValue());
        return;
    }

    _climber.SetTargetPosition(UP_SEQ_1_HEIGHT);  // go up
}

void RaiseTeleArmCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _sensorControl.SetCurrentlyClimbing(false);
    _sensorControl.SetAutoClimbing(false);
}

bool RaiseTeleArmCommand::IsFinished()
{
    return (fabs(_climber.GetEncoderValue() - UP_SEQ_1_HEIGHT) < CLIMBER_TOLERANCE) ||
           fabs(_sensorControl.GetCurTime() - _startTime) > UP_SEQ_TIMEOUT;
}