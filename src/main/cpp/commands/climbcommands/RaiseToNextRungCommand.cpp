#include "commands/climbcommands/RaiseToNextRungCommand.h"

#include <iostream>

#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"
#include "utils/HEIGHTS.h"

RaiseToNextRungCommand::RaiseToNextRungCommand(Climber& climber, SensorBoard& sensorControl)
    : _climber{climber}, _sensorControl{sensorControl}
{
    AddRequirements({&climber});
}

void RaiseToNextRungCommand::Initialize()
{
    _sensorControl.SetDesiredColor(3);
    _sensorControl.SetAutoClimbing(true);
    _sensorControl.SetCurrentlyClimbing(true);

    _turretGood = false;
    _startTime  = _sensorControl.GetCurTime();

    std::cout << " raise to next rung " << std::endl;
}

void RaiseToNextRungCommand::Execute()
{
    _turretGood = _turretGood || _sensorControl.IsAtTurretClimbAngle();
    if (!_turretGood) {
        _startTime = _sensorControl.GetCurTime();
        _climber.SetTargetPosition(_climber.GetEncoderValue());
    } else if ((!_sensorControl.SwingDirJustChanged() && _sensorControl.IsPosSwing()) ||
               fabs(_sensorControl.GetCurTime() - _startTime) > 5.0) {
        // reach to next rung w/ positive swing
        _climber.SetTargetPosition(RAISE_TO_RUNG_SEQ_1_HEIGHT);  // go up
    }
}

void RaiseToNextRungCommand::End(bool interrupted)
{
    std::cout << "done w raise to next rung" << std::endl;
    _sensorControl.SetDesiredColor(0);
    _sensorControl.SetCurrentlyClimbing(false);
    _sensorControl.SetAutoClimbing(false);
}

bool RaiseToNextRungCommand::IsFinished()
{
    return fabs(_climber.GetEncoderValue() - RAISE_TO_RUNG_SEQ_1_HEIGHT) < CLIMBER_TOLERANCE;
}