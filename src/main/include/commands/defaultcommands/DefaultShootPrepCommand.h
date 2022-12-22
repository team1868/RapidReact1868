#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Indexer;
class Shooter;
class SensorBoard;

class DefaultShootPrepCommand
    : public frc2::CommandHelper<frc2::CommandBase, DefaultShootPrepCommand> {
   public:
    explicit DefaultShootPrepCommand(Shooter& shooter,
                                     Indexer& indexer,
                                     SensorBoard& sensorControl);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

   private:
    Shooter& _shooter;
    Indexer& _indexer;
    SensorBoard& _sensorControl;

    double _curTime;
    double _desiredVelocity;
    bool _stuffInElevator;
    double _lastSawBallTime;
};
