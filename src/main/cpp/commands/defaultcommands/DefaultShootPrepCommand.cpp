#include "commands/defaultcommands/DefaultShootPrepCommand.h"

#include <iostream>
#include "microsystems/SensorBoard.h"
#include "subsystems/Indexer.h"
#include "subsystems/Shooter.h"

DefaultShootPrepCommand::DefaultShootPrepCommand(Shooter& shooter,
                                                 Indexer& indexer,
                                                 SensorBoard& sensorControl)
    : _shooter{shooter}, _indexer{indexer}, _sensorControl{sensorControl}
{
    AddRequirements({&shooter});
}

void DefaultShootPrepCommand::Initialize()
{
    _curTime         = _sensorControl.GetCurTime();
    _lastSawBallTime = _curTime - 5.0;

    _shooter.SetHoodHeight(_shooter.CalculateHoodHeight());
}

void DefaultShootPrepCommand::Execute()
{
    _curTime         = _sensorControl.GetCurTime();
    _stuffInElevator = _indexer.GetTopLightSensor() || _indexer.GetBottomLightSensor();

    // TODO fabs may not be needed
    if (_stuffInElevator || fabs(_lastSawBallTime - _curTime) <= 3.0) {
        if (_stuffInElevator) _lastSawBallTime = _curTime;

        _desiredVelocity = _shooter.CalculateFlywheelVelocityDesired();
        _shooter.SetHoodHeight(_shooter.CalculateHoodHeight());
    } else if (_desiredVelocity != 0.0) {
        _desiredVelocity = 0.0;
    }
    _shooter.SetFlywheelRPM(_desiredVelocity);
}

void DefaultShootPrepCommand::End(bool interrupted) {}
