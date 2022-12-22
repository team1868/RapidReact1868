#include "commands/climbcommands/AbortClimbCommand.h"

#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"
#include "utils/CLIMBER_CONSTANTS.h"

AbortClimbCommand::AbortClimbCommand(Climber& climber, SensorBoard& sensorControl)
    : _climber{climber}, _sensorControl{sensorControl}
{
    AddRequirements({&climber});
}

void AbortClimbCommand::Initialize()
{
    _startTime = _sensorControl.GetCurTime();
    _climber.SetPivotArmOutput(RESET_OUTPUT);
    _climber.SetTelescopeArmOutput(RESET_OUTPUT);
    _sensorControl.SetAutoClimbing(true);
}

void AbortClimbCommand::End(bool interrupted) { _sensorControl.SetAutoClimbing(false); }

bool AbortClimbCommand::IsFinished()
{
    return _sensorControl.GetCurTime() - _startTime > ABORT_CLIMB_TIMEOUT;
}