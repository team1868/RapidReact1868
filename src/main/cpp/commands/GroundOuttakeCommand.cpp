#include "commands/GroundOuttakeCommand.h"

#include "microsystems/ControlBoard.h"
#include "subsystems/Indexer.h"
#include "subsystems/Intake.h"
#include "utils/INDEXER_CONSTANTS.h"
#include "utils/INTAKE_CONSTANTS.h"

GroundOuttakeCommand::GroundOuttakeCommand(Intake& intake,
                                           Indexer& indexer,
                                           ControlBoard& humanControl)
    : _intake{intake}, _indexer{indexer}, _humanControl{humanControl}
{
    AddRequirements({&_intake, &_indexer});
}

void GroundOuttakeCommand::Initialize()
{
    _intake.LowerIntakeArm();
    _intake.SetIntakeRollersPower(INTAKE_UNDO_ROLLERS_OUTPUT);
    _indexer.SetIndexRollersPower(INDEX_UNDO_ROLLERS_OUTPUT);
    _indexer.SetElevatorPower(REV_ELEVATOR_OUTPUT);
}

void GroundOuttakeCommand::End(bool interrupted)
{
    _intake.RaiseIntakeArm();
    _intake.SetIntakeRollersPower(RESET_OUTPUT);
    _indexer.SetIndexRollersPower(RESET_OUTPUT);
    _indexer.SetElevatorPower(RESET_OUTPUT);
}

bool GroundOuttakeCommand::IsFinished()
{
    return !_humanControl.GetDesired(Buttons::kGroundOuttakeButton);
}