#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Intake;
class ControlBoard;
class SensorBoard;

class IntakeCommand : public frc2::CommandHelper<frc2::CommandBase, IntakeCommand> {
   public:
    explicit IntakeCommand(Intake& intake, SensorBoard& sensorControl, ControlBoard& humanControl);

    void Initialize() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

   private:
    Intake& _intake;
    SensorBoard& _sensorControl;
    ControlBoard& _humanControl;
};
