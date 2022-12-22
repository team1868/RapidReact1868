#include "subsystems/Blocker.h"

#include <iostream>

#include "microsystems/SensorBoard.h"
#include "subsystems/Turret.h"

Blocker::Blocker(Turret& turret, SensorBoard& sensorControl)
    : _turret{turret}, _sensorControl{sensorControl}
{
}

void Blocker::Periodic()
{
    if (_sensorControl.IsAuto() || _sensorControl.IsAutoClimbing()) {
        std::cout << "Auto or auto climbing, no blocker" << std::endl;
        LowerBlocker();
        _sensorControl.SetBlockerDesired(false);
    } else if (_sensorControl.IsBlockerDesired() && _turret.IsAtTurretZeroAngle()) {
        RaiseBlocker();
    } else {
        LowerBlocker();  // lower while not at zero angle
    }
}

void Blocker::RaiseBlocker() { blockerSolenoid_.Set(true); }

void Blocker::LowerBlocker() { blockerSolenoid_.Set(false); }
