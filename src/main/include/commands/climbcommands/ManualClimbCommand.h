#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

class Climber;
class ControlBoard;
class SensorBoard;

class ManualClimbCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, ManualClimbCommand> {
   public:
    ManualClimbCommand(Climber& _climber, SensorBoard& _sensorControl, ControlBoard& _humanControl);
};