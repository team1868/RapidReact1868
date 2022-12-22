#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Indexer;
class Shooter;
class Turret;
class ControlBoard;
class SensorBoard;

class ShortShotCommand : public frc2::CommandHelper<frc2::CommandBase, ShortShotCommand> {
   public:
    explicit ShortShotCommand(Shooter& shooter,
                              Turret& turret,
                              Indexer& indexer,
                              SensorBoard& sensorControl,
                              ControlBoard& humanControl);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished();

   private:
    Shooter& _shooter;
    Turret& _turret;
    Indexer& _indexer;
    SensorBoard& _sensorControl;
    ControlBoard& _humanControl;
    double _desiredVelocity;
};