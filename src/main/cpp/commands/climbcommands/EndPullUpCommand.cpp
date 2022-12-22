#include "commands/climbcommands/EndPullUpCommand.h"

#include <iostream>

#include "microsystems/ControlBoard.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"
#include "utils/HEIGHTS.h"

EndPullUpCommand::EndPullUpCommand(Climber& climber,
                                   SensorBoard& sensorControl,
                                   ControlBoard& humanControl)
    : _climber{climber}, _sensorControl{sensorControl}, _humanControl{humanControl}
{
    AddRequirements({&climber});
}

void EndPullUpCommand::Initialize()
{
    _startTime             = _sensorControl.GetCurTime();
    _limitSwitchesNowOff   = false;
    _turretGood            = false;
    _finishedCheckingSwing = false;

    _sensorControl.SetAutoClimbing(true);
    _sensorControl.SetCurrentlyClimbing(true);
}

void EndPullUpCommand::Execute()
{
    _sensorControl.SetDesiredColor(0);

    _turretGood = _turretGood || _sensorControl.IsAtTurretClimbAngle();
    if (!_turretGood) {
        _startTime = _sensorControl.GetCurTime();
        _climber.SetTargetPosition(_climber.GetEncoderValue());
        return;
    }

    // swing detection controlled ------
    _finishedCheckingSwing = _finishedCheckingSwing ||
                             _humanControl.GetDesired(Buttons::kHighClimbButton) ||
                             (_sensorControl.SwingDirJustChanged() && !_sensorControl.IsPosSwing());
    if (!_finishedCheckingSwing) { return; }

    // off means engaged!
    bool limitSwitchesTriggered = _climber.GetLRLimitSwitchAND();
    double pivotArmOutput       = RESET_OUTPUT;
    if (limitSwitchesTriggered && _climber.GetHardStopLimitSwitch()) {
        _climber.DisengagePivotHook();
        pivotArmOutput = -0.6;
    }
    _climber.SetPivotArmOutput(pivotArmOutput);

    // limit switches are now off
    if (!_limitSwitchesNowOff && limitSwitchesTriggered) {
        _limitSwitchesNowOff = true;
        _startTime           = _sensorControl.GetCurTime();
    }

    double telescopeArmOutput = RESET_OUTPUT;
    if (_climber.GetEncoderValue() > END_PULL_UP_SEQ_1_HEIGHT) {
        // as long as the encoder value isn't too low, pull up
        telescopeArmOutput = -1.0;
        _climber.DisengagePivotHook();
    }
    _climber.SetTelescopeArmOutput(telescopeArmOutput);
}

void EndPullUpCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _sensorControl.SetCurrentlyClimbing(false);
    _climber.SetTelescopeArmOutput(RESET_OUTPUT);
    _climber.SetPivotArmOutput(RESET_OUTPUT);
    std::cout << "finished end pull up" << std::endl;
    _sensorControl.SetAutoClimbing(false);
}

bool EndPullUpCommand::IsFinished()
{
    return _finishedCheckingSwing && _limitSwitchesNowOff &&
           _sensorControl.GetCurTime() - _startTime > 0.75;
}
