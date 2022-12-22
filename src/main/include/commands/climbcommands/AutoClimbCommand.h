#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

class Climber;
class ControlBoard;
class SensorBoard;

/**
 * A complex auto command that drives forward, releases a hatch, and then drives
 * backward.
 */
class AutoClimbCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, AutoClimbCommand> {
   public:
    /**
     * Creates a new AutoClimbCommand.
     */
    AutoClimbCommand(Climber& _climber, SensorBoard& _sensorControl, ControlBoard& _humanControl);
};