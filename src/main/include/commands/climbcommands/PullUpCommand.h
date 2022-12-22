#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Climber;
class SensorBoard;

class PullUpCommand : public frc2::CommandHelper<frc2::CommandBase, PullUpCommand> {
   public:
    explicit PullUpCommand(Climber& climber, SensorBoard& sensorControl);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

   private:
    Climber& _climber;
    SensorBoard& _sensorControl;

    double _startTime;
    bool _raiseHigher;  // raise a bit more
    bool _resettingPivotArms;
    bool _turretGood;
};
