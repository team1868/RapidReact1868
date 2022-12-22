#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class ControlBoard;
class DriveTrain;

class DefaultDriveCommand : public frc2::CommandHelper<frc2::CommandBase, DefaultDriveCommand> {
   public:
    /**
     * Creates a new DefaultDriveCommand.
     *
     * @param drivetrain The subsystem used by this command.
     */
    explicit DefaultDriveCommand(DriveTrain& drivetrain, ControlBoard& humanControl);

    void Execute() override;

   private:
    DriveTrain& _drivetrain;
    ControlBoard& _humanControl;
};
