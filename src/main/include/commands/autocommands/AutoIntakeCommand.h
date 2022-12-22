#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Indexer;
class Intake;
class SensorBoard;

class AutoIntakeCommand : public frc2::CommandHelper<frc2::CommandBase, AutoIntakeCommand> {
   public:
    explicit AutoIntakeCommand(double time,
                               Intake& intake,
                               Indexer& indexer,
                               SensorBoard& sensorControl);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

   private:
    Intake& _intake;
    Indexer& _indexer;
    SensorBoard& _sensorControl;
    double _timeout;
    double _startTime;
};
