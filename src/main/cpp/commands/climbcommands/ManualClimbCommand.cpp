#include "commands/climbcommands/ManualClimbCommand.h"

#include "commands/climbcommands/CollapseCommand.h"
#include "commands/climbcommands/EndPullUpCommand.h"
#include "commands/climbcommands/PullUpCommand.h"
#include "commands/climbcommands/RaiseToNextRungCommand.h"
#include "commands/climbcommands/ReachBackCommand.h"
#include "microsystems/ControlBoard.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"

ManualClimbCommand::ManualClimbCommand(Climber& _climber,
                                       SensorBoard& _sensorControl,
                                       ControlBoard& _humanControl)
{
    AddCommands(
        // // {Pull Up}
        PullUpCommand(_climber, _sensorControl),
        // {ReachBack}
        ReachBackCommand(_climber, _sensorControl),
        // {RaiseToNextRung}
        RaiseToNextRungCommand(_climber, _sensorControl),
        // {Collapse}
        CollapseCommand(_climber, _sensorControl),
        // {EndPullUp}
        EndPullUpCommand(_climber, _sensorControl, _humanControl));
}