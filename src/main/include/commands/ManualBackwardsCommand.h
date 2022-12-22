#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Indexer;
class Intake;
class ControlBoard;
class SensorBoard;

class ManualBackwardsCommand
    : public frc2::CommandHelper<frc2::CommandBase, ManualBackwardsCommand> {
   public:
    explicit ManualBackwardsCommand(Intake& intake,
                                    Indexer& indexer,
                                    SensorBoard& sensorControl,
                                    ControlBoard& humanControl);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

   private:
    Intake& _intake;
    Indexer& _indexer;
    SensorBoard& _sensorControl;
    ControlBoard& _humanControl;
};
