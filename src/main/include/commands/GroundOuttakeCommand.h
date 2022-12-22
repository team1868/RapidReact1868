#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Indexer;
class Intake;
class ControlBoard;

class GroundOuttakeCommand : public frc2::CommandHelper<frc2::CommandBase, GroundOuttakeCommand> {
   public:
    explicit GroundOuttakeCommand(Intake& intake, Indexer& indexer, ControlBoard& humanControl);

    void Initialize() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

   private:
    Intake& _intake;
    Indexer& _indexer;
    ControlBoard& _humanControl;
    double _startTime, _curTime;
};
