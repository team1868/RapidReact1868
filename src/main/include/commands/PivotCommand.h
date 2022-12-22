#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class DriveTrain;
class SensorBoard;

class PivotCommand : public frc2::CommandHelper<frc2::CommandBase, PivotCommand> {
   public:
    explicit PivotCommand(DriveTrain& drivetrain, SensorBoard& sensorControl, double desiredAngle);

    void Initialize() override;
    void Execute() override;
    bool IsFinished() override;
    void End(bool interrupted) override;

   private:
    DriveTrain& _drivetrain;
    SensorBoard& _sensorControl;

    double _pivotCommandStartTime;

    frc::PIDController _pivotPID{0.015, 0.0, 0.0};

    double const _pivotTimeoutSec = 4.0;
    double const _turnTolerance   = 2.5;
};
