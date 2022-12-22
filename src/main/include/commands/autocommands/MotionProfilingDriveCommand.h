#pragma once

#include <frc2/command/RamseteCommand.h>

class DriveTrain;

frc2::RamseteCommand MPDriveCommand(DriveTrain& drivetrain,
                                    frc::Pose2d m,
                                    bool reverse,
                                    bool fastacc);

frc2::RamseteCommand MPDriveCommand(
    DriveTrain& drivetrain, frc::Pose2d m, frc::Translation2d trans, bool reverse, bool fastacc);
