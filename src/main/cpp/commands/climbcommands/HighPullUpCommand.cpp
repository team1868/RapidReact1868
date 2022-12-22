#include "commands/climbcommands/HighPullUpCommand.h"

#include "microsystems/ControlBoard.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"
#include "utils/HEIGHTS.h"

HighPullUpCommand::HighPullUpCommand(Climber& climber,
                                     SensorBoard& sensorControl,
                                     ControlBoard& humanControl)
    : _climber{climber}, _sensorControl{sensorControl}, _humanControl{humanControl}
{
    AddRequirements({&climber});
}

void HighPullUpCommand::Initialize()
{
    _startTime          = _sensorControl.GetCurTime();
    _limitSwitchesSeen  = false;
    _resettingPivotArms = true;
    _turretGood         = false;

    _sensorControl.SetAutoClimbing(true);
    _sensorControl.SetDesiredColor(7);
}

void HighPullUpCommand::Execute()
{
    _turretGood = _turretGood || _sensorControl.IsAtTurretClimbAngle();
    if (!_turretGood) {
        _startTime = _sensorControl.GetCurTime();
        _climber.SetTargetPosition(_climber.GetEncoderValue());
        return;
    }
    _sensorControl.SetCurrentlyClimbing(true);

    _resettingPivotArms       = _resettingPivotArms && _climber.GetHardStopLimitSwitch();
    double pivotArmOutput     = RESET_OUTPUT;
    double telescopeArmOutput = RESET_OUTPUT;
    if (_resettingPivotArms) { pivotArmOutput = -0.6; }

    if (!_limitSwitchesSeen &&
        (!_climber.GetLeftLimitSwitch() || !_climber.GetRightLimitSwitch())) {
        _limitSwitchesSeen = true;
        _startTime         = _sensorControl.GetCurTime();
    } else if ((!_limitSwitchesSeen &&
                _climber.GetTeleHardStopLimitSwitch()) /* || !stoppedByPivotArms */) {
        telescopeArmOutput = -1.0;
    }

    _climber.SetPivotArmOutput(pivotArmOutput);
    _climber.SetTelescopeArmOutput(telescopeArmOutput);
    // 0.75 was WAY too short. the robot fell trying to pull up onto high rung.
    if (_limitSwitchesSeen && fabs(_sensorControl.GetCurTime() - _startTime) > 7.0) {
        // pause before disengage telescoping arm
        _climber.SetTargetPosition(PULL_UP_SEQ_2_HEIGHT);
    }
}

void HighPullUpCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _climber.SetTelescopeArmOutput(RESET_OUTPUT);
    _sensorControl.SetCurrentlyClimbing(false);
    _sensorControl.SetAutoClimbing(false);
}

bool HighPullUpCommand::IsFinished()
{
    return _climber.GetTargetPosition() == PULL_UP_SEQ_2_HEIGHT &&
           fabs(_climber.GetEncoderValue() - PULL_UP_SEQ_2_HEIGHT) < CLIMBER_TOLERANCE;
}