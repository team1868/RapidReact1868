#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Climber;
class ControlBoard;
class SensorBoard;

class EndPullUpCommand : public frc2::CommandHelper<frc2::CommandBase, EndPullUpCommand> {
   public:
    explicit EndPullUpCommand(Climber& climber,
                              SensorBoard& sensorControl,
                              ControlBoard& humanControl);

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

   private:
    Climber& _climber;
    SensorBoard& _sensorControl;
    ControlBoard& _humanControl;

    double _startTime;
    bool _limitSwitchesNowOff;
    bool _turretGood;
    bool _finishedCheckingSwing;
};
