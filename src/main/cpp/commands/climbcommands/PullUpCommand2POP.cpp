#include "commands/climbcommands/PullUpCommand2POP.h"

#include <iostream>

#include "microsystems/ControlBoard.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"
#include "utils/HEIGHTS.h"

PullUpCommand2POP::PullUpCommand2POP(Climber& climber, SensorBoard& sensorControl)
    : _climber{climber}, _sensorControl{sensorControl}
{
    AddRequirements({&climber});
}

void PullUpCommand2POP::Initialize()
{
    _startTime  = _sensorControl.GetCurTime();
    _turretGood = false;
    std::cout << " pull up 2 POP " << std::endl;

    _sensorControl.SetAutoClimbing(true);
    _sensorControl.SetDesiredColor(1);
}

void PullUpCommand2POP::Execute()
{
    double targetPosition = PULL_UP_SEQ_2_HEIGHT;
    _turretGood           = _turretGood || _sensorControl.IsAtTurretClimbAngle();
    if (!_turretGood) {
        _startTime     = _sensorControl.GetCurTime();
        targetPosition = _climber.GetEncoderValue();
    } else {
        _sensorControl.SetCurrentlyClimbing(true);
    }
    _climber.SetTargetPosition(targetPosition);
}

void PullUpCommand2POP::End(bool interrupted)
{
    _climber.SetTelescopeArmOutput(RESET_OUTPUT);
    _sensorControl.SetDesiredColor(0);
    _sensorControl.SetCurrentlyClimbing(false);
    _sensorControl.SetAutoClimbing(false);
}

bool PullUpCommand2POP::IsFinished()
{
    return _turretGood &&
           fabs(_climber.GetEncoderValue() - PULL_UP_SEQ_2_HEIGHT) < CLIMBER_TOLERANCE;
}