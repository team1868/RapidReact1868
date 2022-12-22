#include "commands/climbcommands/ReachBackCommand.h"

#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"
#include "utils/HEIGHTS.h"

ReachBackCommand::ReachBackCommand(Climber& climber, SensorBoard& sensorControl)
    : _climber{climber}, _sensorControl{sensorControl}
{
    AddRequirements({&climber});
}

void ReachBackCommand::Initialize()
{
    _sensorControl.SetDesiredColor(2);
    _sensorControl.SetAutoClimbing(true);
    _turretGood = false;
}

void ReachBackCommand::Execute()
{
    _turretGood = _turretGood || _sensorControl.IsAtTurretClimbAngle();
    if (!_turretGood) {
        _climber.SetPivotArmTargetPosition(_climber.GetPivotArmEncoderValue());
        _climber.SetTargetPosition(_climber.GetEncoderValue());
        return;
    }
    _sensorControl.SetCurrentlyClimbing(true);

    if (_sensorControl.SwingDirJustChanged() && !_sensorControl.IsPosSwing()) {
        // reach back when changing from positive to negative swing
        _climber.SetPivotArmTargetPosition(REACH_BACK_SEQ_1_LENGTH);
    }

    _climber.SetTargetPosition(REACH_BACK_SEQ_1_TELE_LENGTH);
}

void ReachBackCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _sensorControl.SetCurrentlyClimbing(false);
    _sensorControl.SetAutoClimbing(false);
}

bool ReachBackCommand::IsFinished()
{
    return _turretGood &&
           fabs(_climber.GetEncoderValue() - REACH_BACK_SEQ_1_TELE_LENGTH) < CLIMBER_TOLERANCE &&
           fabs(_climber.GetPivotArmEncoderValue() - REACH_BACK_SEQ_1_LENGTH) < PIVOT_ARM_TOLERANCE;
}