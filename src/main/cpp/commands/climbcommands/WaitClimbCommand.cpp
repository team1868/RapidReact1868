#include "commands/climbcommands/WaitClimbCommand.h"

#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"

WaitClimbCommand::WaitClimbCommand(double waitTime, Climber& climber, SensorBoard& sensorControl)
    : _climber{climber}, _sensorControl{sensorControl}, _waitTime{waitTime}
{
    AddRequirements({&climber});
}

void WaitClimbCommand::Initialize()
{
    _startTime = _sensorControl.GetCurTime();
    _climber.SetPivotArmOutput(RESET_OUTPUT);
    _climber.SetTelescopeArmOutput(RESET_OUTPUT);
    _sensorControl.SetAutoClimbing(true);
}

void WaitClimbCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _sensorControl.SetAutoClimbing(false);
}

bool WaitClimbCommand::IsFinished()
{
    return fabs(_sensorControl.GetCurTime() - _startTime) > _waitTime;
}