#include "commands/climbcommands/HaltPullUpCommand.h"

#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"

HaltPullUpCommand::HaltPullUpCommand(Climber& climber, SensorBoard& sensorControl)
    : _climber{climber}, _sensorControl{sensorControl}
{
    AddRequirements({&climber});
}

void HaltPullUpCommand::Initialize()
{
    _resettingPivotArms = true;
    _climber.DisengagePivotHook();
    _sensorControl.SetAutoClimbing(true);
    _sensorControl.SetCurrentlyClimbing(true);
}

void HaltPullUpCommand::Execute()
{
    _resettingPivotArms = _resettingPivotArms && _climber.GetHardStopLimitSwitch();
    _climber.SetPivotArmOutput(_resettingPivotArms ? -0.6 : RESET_OUTPUT);

    _climber.SetTelescopeArmOutput(
        !_resettingPivotArms && _climber.GetLRLimitSwitchAND() &&
                (_climber.GetTeleHardStopLimitSwitch() /* || !stoppedByPivotArms */)
            ? -1.0
            : RESET_OUTPUT);
}

void HaltPullUpCommand::End(bool interrupted)
{
    _climber.SetTelescopeArmOutput(RESET_OUTPUT);
    _climber.EngagePivotHook();
    _sensorControl.SetCurrentlyClimbing(false);
    _sensorControl.SetAutoClimbing(false);
}

bool HaltPullUpCommand::IsFinished()
{
    return !_resettingPivotArms &&
           (!_climber.GetLeftLimitSwitch() || !_climber.GetRightLimitSwitch());
}