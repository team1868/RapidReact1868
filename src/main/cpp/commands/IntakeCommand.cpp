#include "commands/IntakeCommand.h"

#include "microsystems/ControlBoard.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Intake.h"
#include "utils/INTAKE_CONSTANTS.h"

IntakeCommand::IntakeCommand(Intake& intake, SensorBoard& sensorControl, ControlBoard& humanControl)
    : _intake{intake}, _sensorControl{sensorControl}, _humanControl{humanControl}
{
    AddRequirements({&_intake});
}

void IntakeCommand::Initialize()
{
    _sensorControl.SetDesiredColor(3);
    _intake.SetIntakeRollersPower(INTAKE_ROLLERS_OUTPUT);
    _intake.LowerIntakeArm();
    _sensorControl.SetIntaking(true);
}

void IntakeCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _intake.RaiseIntakeArm();
    _intake.SetIntakeRollersPower(RESET_OUTPUT);
    _sensorControl.SetIntaking(false);
}

bool IntakeCommand::IsFinished() { return !_humanControl.GetDesired(Buttons::kIntakeRightButton); }
