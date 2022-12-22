#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/PIDCommand.h>

class Climber;
class Turret;
class SensorBoard;

class ZeroTurretCommand : public frc2::CommandHelper<frc2::CommandBase, ZeroTurretCommand> {
   public:
    explicit ZeroTurretCommand(Turret& turret, Climber& climber, SensorBoard& sensorControl);

    void Initialize() override;
    void Execute() override;

    void End(bool interrupted) override;
    bool IsFinished();

   private:
    Turret& _turret;
    Climber& _climber;
    SensorBoard& _sensorControl;
};