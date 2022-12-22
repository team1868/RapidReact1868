#include "commands/autocommands/TestDriveAuto.h"

#include "commands/DriveStraightCommand.h"
#include "subsystems/DriveTrain.h"
#include "utils/AUTO_CONSTANTS.h"

using namespace AutoConstants;

TestDriveAuto::TestDriveAuto(DriveTrain& drivetrain)
{
    AddCommands(
        // Drive forward the specified distance
        DriveStraightCommand(drivetrain, kAutoDriveSpeed, kAutoDriveDistanceInches),
        DriveStraightCommand(drivetrain, -kAutoDriveSpeed, kAutoBackupDistanceInches));
}
