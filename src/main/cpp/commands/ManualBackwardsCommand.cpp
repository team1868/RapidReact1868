#include "commands/ManualBackwardsCommand.h"

#include "microsystems/ControlBoard.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Indexer.h"
#include "subsystems/Intake.h"
#include "utils/INDEXER_CONSTANTS.h"
#include "utils/INTAKE_CONSTANTS.h"

ManualBackwardsCommand::ManualBackwardsCommand(Intake& intake,
                                               Indexer& indexer,
                                               SensorBoard& sensorControl,
                                               ControlBoard& humanControl)
    : _intake{intake}, _indexer{indexer}, _sensorControl{sensorControl}, _humanControl{humanControl}
{
    AddRequirements({&_intake, &_indexer});
}

void ManualBackwardsCommand::Initialize()
{
    _sensorControl.SetDesiredColor(8);
    _intake.LowerIntakeArm();
    _intake.SetIntakeRollersPower(INTAKE_UNDO_ROLLERS_OUTPUT);
    _indexer.SetIndexRollersPower(INDEX_UNDO_ROLLERS_OUTPUT);
    _indexer.SetElevatorPower(REV_ELEVATOR_OUTPUT);
}

void ManualBackwardsCommand::Execute() {}

void ManualBackwardsCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _intake.RaiseIntakeArm();
    _intake.SetIntakeRollersPower(RESET_OUTPUT);
    _indexer.SetIndexRollersPower(RESET_OUTPUT);
    _indexer.SetElevatorPower(RESET_OUTPUT);
}

bool ManualBackwardsCommand::IsFinished()
{
    return !_humanControl.GetDesired(Buttons::kOuttakeLeftButton);
}