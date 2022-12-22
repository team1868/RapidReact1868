#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Climber;
class ControlBoard;
class SensorBoard;

class ManualPivotArmCommand : public frc2::CommandHelper<frc2::CommandBase, ManualPivotArmCommand> {
   public:
    explicit ManualPivotArmCommand(Climber& climber,
                                   SensorBoard& sensorControl,
                                   ControlBoard& humanControl);

    void Execute() override;

    void Initialize() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

   private:
    Climber& _climber;
    SensorBoard& _sensorControl;
    ControlBoard& _humanControl;
    bool _turretGood;
};
