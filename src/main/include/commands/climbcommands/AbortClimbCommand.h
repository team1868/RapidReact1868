#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Climber;
class SensorBoard;

class AbortClimbCommand : public frc2::CommandHelper<frc2::CommandBase, AbortClimbCommand> {
   public:
    explicit AbortClimbCommand(Climber& climber, SensorBoard& sensorControl);

    void Initialize() override;
    void End(bool interrupted) override;
    bool IsFinished();

   private:
    Climber& _climber;
    SensorBoard& _sensorControl;
    double _startTime;
};