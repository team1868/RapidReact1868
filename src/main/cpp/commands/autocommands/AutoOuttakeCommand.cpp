#include "commands/autocommands/AutoOuttakeCommand.h"

#include "microsystems/SensorBoard.h"
#include "subsystems/Indexer.h"
#include "subsystems/Intake.h"
#include "utils/INDEXER_CONSTANTS.h"
#include "utils/INTAKE_CONSTANTS.h"

AutoOuttakeCommand::AutoOuttakeCommand(double time,
                                       Intake& intake,
                                       Indexer& indexer,
                                       SensorBoard& sensorControl)
    : _intake{intake}, _indexer{indexer}, _sensorControl{sensorControl}, _timeout{time}
{
    AddRequirements({&_intake, &_indexer});
}

void AutoOuttakeCommand::Initialize()
{
    _startTime = _sensorControl.GetCurTime();
    _intake.LowerIntakeArm();
    _intake.SetIntakeRollersPower(REV_INTAKE_ROLLERS_OUTPUT);
    _indexer.SetIndexRollersPower(INDEX_UNDO_ROLLERS_OUTPUT);
    _indexer.SetElevatorPower(REV_ELEVATOR_OUTPUT);
}

void AutoOuttakeCommand::Execute() {}

void AutoOuttakeCommand::End(bool interrupted)
{
    _intake.RaiseIntakeArm();
    _intake.SetIntakeRollersPower(RESET_OUTPUT);
    _indexer.SetIndexRollersPower(RESET_OUTPUT);
    _indexer.SetElevatorPower(RESET_OUTPUT);
}

bool AutoOuttakeCommand::IsFinished()
{
    return fabs(_sensorControl.GetCurTime() - _startTime) > _timeout;
}