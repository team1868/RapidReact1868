#include "commands/autocommands/AutoSequences.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <math.h>

#include "commands/IntakeCommand.h"
#include "commands/PivotCommand.h"
#include "commands/ShootCommand.h"
#include "commands/autocommands/AutoIntakeCommand.h"
#include "commands/autocommands/AutoOuttakeCommand.h"
#include "commands/autocommands/AutoShootCommand.h"
#include "commands/autocommands/MotionProfilingDriveCommand.h"
#include "commands/defaultcommands/DefaultIndexCommand.h"
#include "commands/turretcommands/DefaultTurretNewCommand.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/Indexer.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/Turret.h"
#include "utils/AUTO_CONSTANTS.h"
#include "utils/AUTO_POINTS.h"

using namespace AutoConstants;

frc::Pose2d const zeroPose{0_m, 0_m, 0_deg};

TwoBaller::TwoBaller(DriveTrain& drivetrain,
                     Intake& intake,
                     Shooter& shooter,
                     SensorBoard& sensorControl,
                     Turret& turret,
                     Indexer& indexer,
                     frc2::InstantCommand& stopDrivetrainCommand,
                     DefaultIndexCommand& defaultIndexCommand)
{
    printf("\n\n2 baller\n\n");

    AddCommands(frc2::InstantCommand([&]() { turret.SetAutoFreezeTurret(true); }),
                frc2::InstantCommand([&]() { shooter.SetHoodHeight(61.876); }),
                frc2::InstantCommand([&]() { drivetrain.ResetOdometry(zeroPose); }),
                frc2::ParallelRaceGroup(AutoIntakeCommand(4.0, intake, indexer, sensorControl),
                                        MPDriveCommand(drivetrain, TWO_BALL_POS_1, false, false),
                                        defaultIndexCommand),
                stopDrivetrainCommand,
                // shoot ball 1+2
                AutoShootCommand(3.0, false, false, -0.0, shooter, indexer, sensorControl));
}

TwoBallDefense::TwoBallDefense(DriveTrain& drivetrain,
                               Intake& intake,
                               Shooter& shooter,
                               SensorBoard& sensorControl,
                               Turret& turret,
                               Indexer& indexer,
                               frc2::InstantCommand& stopDrivetrainCommand,
                               DefaultIndexCommand& defaultIndexCommand)
{
    printf("\n\n2 ball DEFENSE \n\n");

    AddCommands(
        frc2::InstantCommand([&]() { turret.SetAutoFreezeTurret(true); }),
        frc2::InstantCommand([&]() { shooter.SetHoodHeight(61.876); }),
        frc2::InstantCommand([&]() { drivetrain.ResetOdometry(zeroPose); }),
        frc2::ParallelRaceGroup(AutoIntakeCommand(4.0, intake, indexer, sensorControl),
                                MPDriveCommand(drivetrain, TWO_BALL_DEF_POS_1, false, false),
                                defaultIndexCommand),
        stopDrivetrainCommand,
        frc2::InstantCommand([&]() { turret.SetAutoFreezeTurret(false); }),
        // shoot ball 1+2
        AutoShootCommand(3.0, false, false, -0.0, shooter, indexer, sensorControl),
        // intake other alliance ball
        frc2::ParallelRaceGroup(
            AutoIntakeCommand(4.0, intake, indexer, sensorControl),
            MPDriveCommand(
                drivetrain, TWO_BALL_DEF_POS_2, TWO_BALL_DEF_POS_2_WAYPOINT, false, false),
            defaultIndexCommand),
        stopDrivetrainCommand,
        // turn around and outtake into hangar
        MPDriveCommand(drivetrain, TWO_BALL_DEF_POS_3, false, false),
        stopDrivetrainCommand,
        AutoOuttakeCommand(1.5, intake, indexer, sensorControl));
}

ThreeBallHP::ThreeBallHP(DriveTrain& drivetrain,
                         Intake& intake,
                         Shooter& shooter,
                         SensorBoard& sensorControl,
                         Turret& turret,
                         Indexer& indexer,
                         frc2::InstantCommand& stopDrivetrainCommand,
                         DefaultIndexCommand& defaultIndexCommand,
                         DefaultTurretNewCommand& defaultTurretNewCommand)
{
    printf("\n\n3 baller short towards HP\n\n");

    AddCommands(
        frc2::InstantCommand([&]() { shooter.SetHoodHeight(61.876); }),
        frc2::InstantCommand([&]() { drivetrain.ResetOdometry(zeroPose); }),
        frc2::ParallelRaceGroup(AutoIntakeCommand(4.0, intake, indexer, sensorControl),
                                MPDriveCommand(drivetrain, {3.62_m, 0.0_m, 0.0_deg}, false, false),
                                defaultIndexCommand),
        stopDrivetrainCommand,
        frc2::InstantCommand([&]() { drivetrain.ResetOdometry(zeroPose); }),
        MPDriveCommand(drivetrain, {-0.8_m, 0.0_m, 0.0_deg}, true, false),
        // shoot ball 1+2
        AutoShootCommand(3.0, false, false, -0.0, shooter, indexer, sensorControl),
        frc2::InstantCommand([&]() { drivetrain.ResetOdometry(zeroPose); }),
        // towards HP
        frc2::ParallelRaceGroup(
            AutoIntakeCommand(6.0, intake, indexer, sensorControl),
            MPDriveCommand(drivetrain, {3.0_m, 3.0_m * -0.2857, -16.6_deg}, false, false),
            defaultTurretNewCommand,
            defaultIndexCommand),
        frc2::InstantCommand([&]() { drivetrain.ResetOdometry(zeroPose); }),
        stopDrivetrainCommand,
        // driveback closer to hub
        frc2::ParallelRaceGroup(MPDriveCommand(drivetrain, {-3.0_m, 0.0_m, 0.0_deg}, true, false),
                                defaultTurretNewCommand),
        stopDrivetrainCommand,
        // shoot ball 3
        AutoShootCommand(3.0, false, false, -0.0, shooter, indexer, sensorControl));
}

// take in angle in degrees
ThreeBallKMM::ThreeBallKMM(DriveTrain& drivetrain,
                           Intake& intake,
                           Shooter& shooter,
                           SensorBoard& sensorControl,
                           Turret& turret,
                           Indexer& indexer,
                           frc2::InstantCommand& stopDrivetrainCommand)
{
    printf("\n\n3ballkmm \n\n");
    AddCommands(
        frc2::ParallelCommandGroup(
            AutoIntakeCommand(4.0, intake, indexer, sensorControl),
            frc2::InstantCommand([&]() { drivetrain.ResetOdometry(zeroPose); }, {}),
            MPDriveCommand(drivetrain, {3.62_m, 0.0_m, 0.0_deg}, false, false)),
        stopDrivetrainCommand,
        AutoShootCommand(2.0, true, true, 1000, shooter, indexer, sensorControl),
        AutoShootCommand(1.0, true, true, 1000, shooter, indexer, sensorControl),
        frc2::InstantCommand([&]() { drivetrain.ResetOdometry(zeroPose); }),
        MPDriveCommand(
            drivetrain, {-3.0_m / 0.3048, 3.0_m / 0.3048, -(135.0_deg + 90.0_deg)}, false, false),
        // turn; try +135??
        stopDrivetrainCommand,
        AutoShootCommand(2.0, true, true, 1000, shooter, indexer, sensorControl));
}

ThreeBallTriangle::ThreeBallTriangle(DriveTrain& drivetrain,
                                     Intake& intake,
                                     Shooter& shooter,
                                     SensorBoard& sensorControl,
                                     Turret& turret,
                                     Indexer& indexer,
                                     frc2::InstantCommand& stopDrivetrainCommand)
{
    AddCommands(
        frc2::InstantCommand([&]() { drivetrain.ResetOdometry(zeroPose); }),
        frc2::ParallelRaceGroup(AutoIntakeCommand(6.0, intake, indexer, sensorControl),
                                MPDriveCommand(drivetrain, {3.62_m, 0.0_m, 0.0_deg}, false, false)),
        stopDrivetrainCommand,
        AutoShootCommand(2.0, true, true, 1000, shooter, indexer, sensorControl),
        AutoShootCommand(1.0, true, true, 1000, shooter, indexer, sensorControl),
        frc2::InstantCommand([&]() { drivetrain.ResetOdometry(zeroPose); }),
        frc2::ParallelRaceGroup(
            AutoIntakeCommand(6.0, intake, indexer, sensorControl),
            MPDriveCommand(drivetrain,
                           {-9.76_m * 0.8449, -9.76_m * 0.5349, (270.0_deg - 32.24_deg)},
                           false,
                           false)),
        stopDrivetrainCommand,
        AutoShootCommand(2.0, true, true, 1000, shooter, indexer, sensorControl));
}

FourBall::FourBall(DriveTrain& drivetrain,
                   Intake& intake,
                   Shooter& shooter,
                   SensorBoard& sensorControl,
                   Turret& turret,
                   Indexer& indexer,
                   frc2::InstantCommand& stopDrivetrainCommand,
                   DefaultIndexCommand& defaultIndexCommand,
                   DefaultTurretNewCommand& defaultTurretNewCommand)
{
    printf("4 baller\n");

    AddCommands(
        // shoot, intake intake shoot, intake intake shoot
        frc2::InstantCommand([&]() { drivetrain.ResetOdometry(zeroPose); }),
        frc2::InstantCommand([&]() { shooter.SetHoodHeight(61.876); }),
        frc2::ParallelRaceGroup(AutoIntakeCommand(8.0, intake, indexer, sensorControl),
                                MPDriveCommand(drivetrain, FOUR_BALL_POS_1, false, false),
                                defaultIndexCommand,
                                defaultTurretNewCommand),
        // 2nd ball intake
        frc2::ParallelRaceGroup(AutoIntakeCommand(1.0, intake, indexer, sensorControl),
                                defaultIndexCommand),
        stopDrivetrainCommand,
        // shoot ball 1+2
        AutoShootCommand(3.0, false, false, -0.0, shooter, indexer, sensorControl),
        // 3rd ball intake, maybe 4th
        frc2::ParallelRaceGroup(AutoIntakeCommand(8.0, intake, indexer, sensorControl),
                                MPDriveCommand(drivetrain, FOUR_BALL_POS_2, false, false),
                                defaultIndexCommand,
                                defaultTurretNewCommand),
        // 4th ball intake
        frc2::ParallelRaceGroup(AutoIntakeCommand(2.0, intake, indexer, sensorControl),
                                defaultIndexCommand,
                                defaultTurretNewCommand),
        stopDrivetrainCommand,
        frc2::ParallelRaceGroup(MPDriveCommand(drivetrain, FOUR_BALL_POS_3, false, true),
                                defaultIndexCommand,
                                defaultTurretNewCommand),
        // shoot ball 3 and 4
        AutoShootCommand(3.0, false, false, -0.0, shooter, indexer, sensorControl));
}

// take in angle in degrees
FiveBallTrans::FiveBallTrans(DriveTrain& drivetrain,
                             Intake& intake,
                             Shooter& shooter,
                             SensorBoard& sensorControl,
                             Turret& turret,
                             Indexer& indexer,
                             frc2::InstantCommand& stopDrivetrainCommand,
                             DefaultIndexCommand& defaultIndexCommand,
                             DefaultTurretNewCommand& defaultTurretNewCommand)
{
    printf("\n\n5 ball w/trans point\n\n");
    AddCommands(
        // shoot, intake intake shoot, intake intake shoot
        frc2::InstantCommand([&]() { drivetrain.ResetOdometry(zeroPose); }),
        frc2::InstantCommand([&]() { shooter.SetHoodHeight(shooter.CalculateHoodHeight()); }),
        // preloaded ball
        AutoShootCommand(0.7,
                         false,
                         true,
                         shooter.CalculateFlywheelVelocityDesired(),
                         shooter,
                         indexer,
                         sensorControl),
        // 2nd ball intake -- 8.25 ft right = 2nd ball, 3.62 ft forward = 1st ball
        frc2::ParallelRaceGroup(
            AutoIntakeCommand(8.0, intake, indexer, sensorControl),
            MPDriveCommand(drivetrain, {1.0_m, 8.25_m, 85.0_deg}, {3.62_m, 0.0_m}, false, false),
            defaultIndexCommand),
        stopDrivetrainCommand,
        frc2::InstantCommand([&]() { shooter.SetHoodHeight(shooter.CalculateHoodHeight()); }),
        // shoot ball 2 and 3
        AutoShootCommand(1.5,
                         false,
                         true,
                         shooter.CalculateFlywheelVelocityDesired(),
                         shooter,
                         indexer,
                         sensorControl),
        frc2::InstantCommand([&]() { drivetrain.ResetOdometry(zeroPose); }),
        // 4th ball intake, maybe 5th
        frc2::ParallelCommandGroup(
            frc2::ParallelRaceGroup(AutoIntakeCommand(6.0, intake, indexer, sensorControl),
                                    MPDriveCommand(drivetrain, FIVE_BALL_POS_1, false, true),
                                    defaultIndexCommand,
                                    defaultTurretNewCommand),
            frc2::InstantCommand([&]() { shooter.SetHoodHeight(shooter.CalculateHoodHeight()); })),
        // 5th ball intake
        frc2::ParallelRaceGroup(AutoIntakeCommand(1.0, intake, indexer, sensorControl),
                                defaultIndexCommand,
                                defaultTurretNewCommand),
        frc2::InstantCommand([&]() { drivetrain.ResetOdometry(zeroPose); }),
        frc2::ParallelCommandGroup(
            frc2::ParallelRaceGroup(
                MPDriveCommand(drivetrain, {-7.0_m, 0.0_m, 0.0_deg}, true, true),
                defaultIndexCommand,
                defaultTurretNewCommand),
            frc2::InstantCommand([&]() { shooter.SetHoodHeight(shooter.CalculateHoodHeight()); })),
        frc2::InstantCommand([&]() { shooter.SetHoodHeight(shooter.CalculateHoodHeight()); }),
        // shoot ball 4 and 5
        AutoShootCommand(2.0,
                         false,
                         true,
                         shooter.CalculateFlywheelVelocityDesired(),
                         shooter,
                         indexer,
                         sensorControl));
}
