#include "commands/defaultcommands/DefaultDriveCommand.h"

#include "microsystems/ControlBoard.h"
#include "subsystems/DriveTrain.h"

DefaultDriveCommand::DefaultDriveCommand(DriveTrain& drivetrain, ControlBoard& humanControl)
    : _drivetrain{drivetrain}, _humanControl{humanControl}
{
    AddRequirements({&drivetrain});
}

void DefaultDriveCommand::Execute()
{
    _drivetrain.ArcadeDrive(_humanControl.GetRightJoyY(), _humanControl.GetLeftJoyX());
}
