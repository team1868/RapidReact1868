#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class DriveTrain;

class DriveStraightCommand : public frc2::CommandHelper<frc2::CommandBase, DriveStraightCommand> {
   public:
    DriveStraightCommand(DriveTrain& drivetrain, double speed, double distance);
    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

   private:
    DriveTrain& _drivetrain;
    double _speed;
    double _distance;
};
