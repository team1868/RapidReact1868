#include "commands/autocommands/MotionProfilingDriveCommand.h"

#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <units/angle.h>
#include <units/length.h>
#include <iostream>

#include "subsystems/DriveTrain.h"
#include "utils/AUTO_CONSTANTS.h"
#include "utils/DRIVE_CONSTANTS.h"

frc2::RamseteCommand MPDriveCommand(DriveTrain& drivetrain,
                                    frc::Pose2d m,
                                    bool reverse,
                                    bool fastacc)
{
    return MPDriveCommand(drivetrain, m, {0.0_m, 0.0_m}, reverse, fastacc);
}

frc2::RamseteCommand MPDriveCommand(
    DriveTrain& drivetrain, frc::Pose2d m, frc::Translation2d t, bool reverse, bool fastacc)
{
    // voltage constraint -- uses up to 10 volts
    frc::DifferentialDriveVoltageConstraint autoVoltageConstraint(
        frc::SimpleMotorFeedforward<units::meters>(
            DriveConstants::ks, DriveConstants::kv, DriveConstants::ka),
        drivetrain.GetDriveKinematics(),
        10_V);

    frc::CentripetalAccelerationConstraint centripetalConstraint(1_mps_sq);

    // Set up config for trajectory
    frc::TrajectoryConfig config(
        AutoConstants::kMaxSpeed,
        fastacc ? AutoConstants::kMaxFastAcceleration : AutoConstants::kMaxAcceleration);

    // Add kinematics to ensure max speed is actually obeyed and apply voltage constraint
    config.SetKinematics(drivetrain.GetDriveKinematics());
    config.AddConstraint(autoVoltageConstraint);
    config.AddConstraint(centripetalConstraint);

    frc::RamseteController ramseteController =
        frc::RamseteController(AutoConstants::kRamseteB, AutoConstants::kRamseteZeta);
    ramseteController.SetEnabled(true);

    config.SetReversed(reverse);

    auto myTrajectory =
        (t.X() != 0.0_m || t.Y() != 0.0_m)
            ? frc::TrajectoryGenerator::GenerateTrajectory(drivetrain.GetPose(), {t}, m, config)
            : frc::TrajectoryGenerator::GenerateTrajectory(drivetrain.GetPose(), {}, m, config);

    return frc2::RamseteCommand(
        myTrajectory,
        [&]() { return drivetrain.GetPose(); },
        ramseteController,
        drivetrain.GetDriveKinematics(),
        [&](auto left_speed, auto right_speed) {
            drivetrain.SetLeftRightSpeeds(left_speed, -right_speed);
        },
        {&drivetrain});
}
