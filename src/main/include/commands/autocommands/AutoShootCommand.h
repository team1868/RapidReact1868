#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Indexer;
class Shooter;
class SensorBoard;

// NOTE: auto shoot prep command and auto shoot? to save time???
class AutoShootCommand : public frc2::CommandHelper<frc2::CommandBase, AutoShootCommand> {
   public:
    explicit AutoShootCommand(double time,
                              bool isShortShot,
                              bool overrideVel,
                              double hardCodeVel,
                              Shooter& shooter,
                              Indexer& indexer,
                              SensorBoard& sensorControl);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished();

   private:
    double _timeout;
    bool _isShortShot;
    bool _overrideVel;
    double _hardCodeVel;
    Shooter& _shooter;
    Indexer& _indexer;
    SensorBoard& _sensorControl;
    double _startTime;
    bool _hasStartedShooting;
    double _desiredVelocity;
};