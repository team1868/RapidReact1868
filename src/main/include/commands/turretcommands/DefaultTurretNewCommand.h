#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Climber;
class Turret;
class SensorBoard;

class DefaultTurretNewCommand
    : public frc2::CommandHelper<frc2::CommandBase, DefaultTurretNewCommand> {
   public:
    explicit DefaultTurretNewCommand(Turret& turret, Climber& climber, SensorBoard& sensorControl);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    double CheckBounds(double desiredAngle, double currEncAngle);

   private:
    Turret& _turret;
    Climber& _climber;
    SensorBoard& _sensorControl;

    bool _spinUp, _pos, _scanTurretDesired, _oldTargetDesired;
    int _numFramesGone;
    double _numTimesConst;
};