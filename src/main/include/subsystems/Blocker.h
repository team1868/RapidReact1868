#pragma once

#include <frc/Solenoid.h>
#include <frc2/command/SubsystemBase.h>

#include "utils/PORTS.h"

class Turret;
class SensorBoard;

class Blocker : public frc2::SubsystemBase {
   public:
    Blocker(Turret& turret, SensorBoard& sensorControl);

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;
    void RaiseBlocker();
    void LowerBlocker();

   private:
    Turret& _turret;
    SensorBoard& _sensorControl;

    frc::Solenoid blockerSolenoid_{
        PNEUMATICS_MODULE_ID, PNEUMATICS_MODULE_TYPE, BLOCKER_SOLENOID_PORT};
};
