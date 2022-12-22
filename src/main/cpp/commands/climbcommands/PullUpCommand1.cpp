#include "commands/climbcommands/PullUpCommand1.h"

#include <iostream>

#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"

PullUpCommand1::PullUpCommand1(Climber& climber, SensorBoard& sensorControl)
    : _climber{climber}, _sensorControl{sensorControl}
{
    AddRequirements({&climber});
}

void PullUpCommand1::Initialize()
{
    _startTime          = _sensorControl.GetCurTime();
    _resettingPivotArms = true;
    _turretGood         = false;
    _limitSwitchesSeen  = false;
    _climber.DisengagePivotHook();
    std::cout << " pull up " << std::endl;

    _sensorControl.SetAutoClimbing(true);
    _sensorControl.SetDesiredColor(1);
}

void PullUpCommand1::Execute()
{
    // turret in correct position
    _turretGood = _turretGood || _sensorControl.IsAtTurretClimbAngle();
    if (!_turretGood && !_sensorControl.IsAtTurretClimbAngle()) {
        _startTime = _sensorControl.GetCurTime();
        _climber.SetTargetPosition(_climber.GetEncoderValue());
        return;
    }

    _sensorControl.SetCurrentlyClimbing(true);

    // make sure pivot arms are reset first
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

    if (!_sensorControl.SwingDirJustChanged() || _sensorControl.IsPosSwing()) {
        _climber.SetTelescopeArmOutput(RESET_OUTPUT);
        return;
    }

    // off means engaged!
    if ((!_climber.GetLeftLimitSwitch() || !_climber.GetRightLimitSwitch()) &&
        !_limitSwitchesSeen) {
        _climber.SetTelescopeArmOutput(RESET_OUTPUT);
        _climber.EngagePivotHook();
        _limitSwitchesSeen = true;
    }
    _climber.SetTelescopeArmOutput(
        !_limitSwitchesSeen && _climber.GetTeleHardStopLimitSwitch() ? -1.0 : RESET_OUTPUT);
}

void PullUpCommand1::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _sensorControl.SetCurrentlyClimbing(false);
    _sensorControl.SetAutoClimbing(false);
    _climber.SetTelescopeArmOutput(RESET_OUTPUT);
    _climber.SetPivotArmOutput(RESET_OUTPUT);
}

bool PullUpCommand1::IsFinished() { return _limitSwitchesSeen; }
