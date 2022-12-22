#include "commands/autocommands/AutoShootCommand.h"

#include <iostream>

#include "microsystems/SensorBoard.h"
#include "subsystems/Indexer.h"
#include "subsystems/Shooter.h"
#include "utils/INDEXER_CONSTANTS.h"

AutoShootCommand::AutoShootCommand(double time,
                                   bool isShortShot,
                                   bool overrideVel,
                                   double hardCodeVel,
                                   Shooter& shooter,
                                   Indexer& indexer,
                                   SensorBoard& sensorControl)
    : _timeout{time},
      _isShortShot{isShortShot},
      _overrideVel{overrideVel},
      _hardCodeVel{hardCodeVel},
      _shooter{shooter},
      _indexer{indexer},
      _sensorControl{sensorControl}
{
    AddRequirements({&_shooter, &_indexer});
}

void AutoShootCommand::Initialize()
{
    _startTime          = _sensorControl.GetCurTime();
    _hasStartedShooting = false;
    _desiredVelocity    = 0.0;
}

void AutoShootCommand::Execute()
{
    if (_overrideVel) {
        // override velocity
        _desiredVelocity = _hardCodeVel;
    } else if (_isShortShot) {
        _desiredVelocity = 1650;
    } else if (!_hasStartedShooting) {
        _desiredVelocity = _shooter.CalculateFlywheelVelocityDesired();
        _shooter.SetHoodHeight(_shooter.CalculateHoodHeight());
    }

    _shooter.SetFlywheelRPM(_desiredVelocity);

    if (_shooter.IsFlywheelAtSpeed(_desiredVelocity)) {
        _hasStartedShooting = true;
        _sensorControl.SetDesiredColor(2);
        _indexer.SetElevatorPower(ELEVATOR_OUTPUT);
        _indexer.SetIndexRollersPower(INDEX_ROLLERS_OUTPUT);
    } else {
        // not prepped
        _startTime          = _sensorControl.GetCurTime();
        _hasStartedShooting = false;
        _sensorControl.SetDesiredColor(1);
        _indexer.SetElevatorPower(RESET_OUTPUT);
        _indexer.SetIndexRollersPower(RESET_OUTPUT);
    }
}

void AutoShootCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _shooter.SetFlywheelRPM(RESET_OUTPUT);
    _indexer.SetElevatorPower(RESET_OUTPUT);
}

bool AutoShootCommand::IsFinished()
{
    return !_indexer.GetTopLightSensor() && _sensorControl.GetCurTime() - _startTime > _timeout;
}