#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/PIDCommand.h>

class Shooter;
class Turret;
class SensorBoard;

class ClimbTurretCommand : public frc2::CommandHelper<frc2::CommandBase, ClimbTurretCommand> {
   public:
    explicit ClimbTurretCommand(Turret& turret, Shooter& shooter, SensorBoard& sensorControl);

    void Initialize() override;
    void Execute() override;

    void End(bool interrupted) override;
    bool IsFinished();

   private:
    Turret& _turret;
    Shooter& _shooter;
    SensorBoard& _sensorControl;
};