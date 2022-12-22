#include "commands/autocommands/TestSuperAuto.h"

#include "commands/autocommands/AutoIntakeCommand.h"
#include "commands/autocommands/AutoShootCommand.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Indexer.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

TestSuperAuto::TestSuperAuto(Shooter& shooter,
                             Indexer& indexer,
                             Intake& intake,
                             SensorBoard& sensorControl)
{
    AddCommands(
        // Drive forward the specified distance
        AutoIntakeCommand(3.0, intake, indexer, sensorControl),
        AutoShootCommand(3.0, false, true, 1200, shooter, indexer, sensorControl));
}
