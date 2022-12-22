#include "commands/climbcommands/AutoClimbCommand.h"

#include "commands/climbcommands/CollapseCommand.h"
#include "commands/climbcommands/EndPullUpCommand.h"
#include "commands/climbcommands/PullUpCommand.h"
#include "commands/climbcommands/PullUpCommand1.h"
#include "commands/climbcommands/PullUpCommand2POP.h"
#include "commands/climbcommands/RaiseTeleArmCommand.h"
#include "commands/climbcommands/RaiseToNextRungCommand.h"
#include "commands/climbcommands/ReachBackCommand.h"
#include "commands/climbcommands/WaitClimbCommand.h"
#include "microsystems/ControlBoard.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"

AutoClimbCommand::AutoClimbCommand(Climber& _climber,
                                   SensorBoard& _sensorControl,
                                   ControlBoard& _humanControl)
{
    AddCommands(

        // {PullUp}
        PullUpCommand(_climber, _sensorControl),
        // {ReachBack}
        ReachBackCommand(_climber, _sensorControl),
        // {RaiseToNextRung}
        RaiseToNextRungCommand(_climber, _sensorControl),
        // {Collapse}
        CollapseCommand(_climber, _sensorControl),
        // {EndPulUp}
        EndPullUpCommand(_climber, _sensorControl, _humanControl),

        // wait 5 seconds to stop swinging
        // DON'T TOUCH THIS AHHHHHHH
        WaitClimbCommand(3.0, _climber, _sensorControl),

        // {Pull Up1}
        PullUpCommand1(_climber, _sensorControl),

        // wait 10 seconds to stop swinging
        WaitClimbCommand(2.0, _climber, _sensorControl),

        // {Pull Up2 POP OFF}
        PullUpCommand2POP(_climber, _sensorControl),

        // {ReachBack}
        ReachBackCommand(_climber, _sensorControl),
        // {RaiseToNextRung}
        RaiseToNextRungCommand(_climber, _sensorControl),
        // {Collapse}
        CollapseCommand(_climber, _sensorControl),
        // {EndPullUp}
        EndPullUpCommand(_climber, _sensorControl, _humanControl));
}