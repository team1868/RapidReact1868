#include "commands/climbcommands/PullUpCommand.h"

#include <iostream>

#include "microsystems/ControlBoard.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"
#include "utils/HEIGHTS.h"

PullUpCommand::PullUpCommand(Climber& climber, SensorBoard& sensorControl)
    : _climber{climber}, _sensorControl{sensorControl}
{
    AddRequirements({&climber});
}

void PullUpCommand::Initialize()
{
    _sensorControl.SetDesiredColor(1);
    _startTime          = _sensorControl.GetCurTime();
    _raiseHigher        = false;
    _resettingPivotArms = true;
    _turretGood         = false;
    _climber.DisengagePivotHook();
    std::cout << " pull up " << std::endl;

    _sensorControl.SetAutoClimbing(true);
}

void PullUpCommand::Execute()
{
    _turretGood = _turretGood || _sensorControl.IsAtTurretClimbAngle();
    if (!_turretGood) {
        _startTime = _sensorControl.GetCurTime();
        _climber.SetTargetPosition(_climber.GetEncoderValue());
        return;
    }

    _sensorControl.SetCurrentlyClimbing(true);

    if (_resettingPivotArms) {
        if (_climber.GetHardStopLimitSwitch()) {
            _climber.SetPivotArmOutput(-0.6);
            _climber.SetTelescopeArmOutput(RESET_OUTPUT);
            return;
        } else {
            _resettingPivotArms = false;
            _climber.SetPivotArmOutput(RESET_OUTPUT);
        }
    }

    // off means engaged!
    _raiseHigher = _raiseHigher || _climber.GetLRLimitSwitchAND();
    if (!_raiseHigher) {
        _startTime = _sensorControl.GetCurTime();
        _climber.EngagePivotHook();
    }

    _climber.SetTelescopeArmOutput(
        !_raiseHigher && _climber.GetTeleHardStopLimitSwitch() ? -1.0 : RESET_OUTPUT);

    if (_raiseHigher && fabs(_sensorControl.GetCurTime() - _startTime) > 1.0) {
        _climber.SetTargetPosition(PULL_UP_SEQ_2_HEIGHT);
    }
}

void PullUpCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _sensorControl.SetCurrentlyClimbing(false);
    _sensorControl.SetAutoClimbing(false);
    _climber.SetTelescopeArmOutput(RESET_OUTPUT);
}

bool PullUpCommand::IsFinished()
{
    return _climber.GetTargetPosition() == PULL_UP_SEQ_2_HEIGHT &&
           fabs(_climber.GetEncoderValue() - PULL_UP_SEQ_2_HEIGHT) < CLIMBER_TOLERANCE;
}
