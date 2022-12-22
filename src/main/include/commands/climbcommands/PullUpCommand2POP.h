#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Climber;
class SensorBoard;

class PullUpCommand2POP : public frc2::CommandHelper<frc2::CommandBase, PullUpCommand2POP> {
   public:
    explicit PullUpCommand2POP(Climber& climber, SensorBoard& sensorControl);

    void Initialize() override;

    void Execute() override;

    void End(bool interrupted) override;

    bool IsFinished() override;

   private:
    Climber& _climber;
    SensorBoard& _sensorControl;

    double _startTime;
    bool _turretGood;
};
