#include "commands/autocommands/AutoIntakeCommand.h"

#include "microsystems/SensorBoard.h"
#include "subsystems/Indexer.h"
#include "subsystems/Intake.h"
#include "utils/INTAKE_CONSTANTS.h"

AutoIntakeCommand::AutoIntakeCommand(double time,
                                     Intake& intake,
                                     Indexer& indexer,
                                     SensorBoard& sensorControl)
    : _intake{intake}, _indexer{indexer}, _sensorControl{sensorControl}, _timeout{time}
{
    AddRequirements({&_intake});
}

void AutoIntakeCommand::Initialize()
{
    _startTime = _sensorControl.GetCurTime();
    _intake.SetIntakeRollersPower(INTAKE_ROLLERS_OUTPUT);
    _intake.LowerIntakeArm();
    _sensorControl.SetIntaking(true);
}

void AutoIntakeCommand::Execute() {}

void AutoIntakeCommand::End(bool interrupted)
{
    _intake.RaiseIntakeArm();
    _intake.SetIntakeRollersPower(RESET_OUTPUT);
    _sensorControl.SetIntaking(false);
}

bool AutoIntakeCommand::IsFinished()
{
    // finish if full or defined timeout
    return (_sensorControl.GetCurTime() - _startTime > _timeout) ||
           (_indexer.GetTopLightSensor() && _indexer.GetBottomLightSensor());
}
