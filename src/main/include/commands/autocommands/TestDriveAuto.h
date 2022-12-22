#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

class DriveTrain;

/**
 * drives forward
 *
 */
class TestDriveAuto : public frc2::CommandHelper<frc2::SequentialCommandGroup, TestDriveAuto> {
   public:
    TestDriveAuto(DriveTrain& drivetrain);
};
