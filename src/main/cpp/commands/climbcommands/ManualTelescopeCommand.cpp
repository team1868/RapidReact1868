#include "commands/climbcommands/ManualTelescopeCommand.h"

#include "microsystems/ControlBoard.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"
#include "utils/CLIMBER_CONSTANTS.h"
#include "utils/HEIGHTS.h"

ManualTelescopeCommand::ManualTelescopeCommand(Climber& climber,
                                               SensorBoard& sensorControl,
                                               ControlBoard& humanControl)
    : _climber{climber}, _sensorControl{sensorControl}, _humanControl{humanControl}
{
    AddRequirements({&climber});
}

void ManualTelescopeCommand::Initialize()
{
    _turretGood = false;
    _sensorControl.SetDesiredColor(7);
}

void ManualTelescopeCommand::Execute()
{
    double telescopeOutput = RESET_OUTPUT;
    _turretGood            = _turretGood || _sensorControl.IsAtTurretClimbAngle();
    if (_turretGood && _humanControl.GetDesired(Buttons::kTeleArmDownButton) &&
        _climber.GetTeleHardStopLimitSwitch()) {
        telescopeOutput = _climber.GetResetClimbEncoders() ? TELESCOPE_DOWN_OUTPUT : -0.5;
    } else if (_turretGood && _humanControl.GetDesired(Buttons::kTeleArmUpButton) &&
               (_climber.GetResetClimbEncoders() ||
                _climber.GetEncoderValue() <= TELESCOPE_UPPER_LIMIT)) {
        telescopeOutput = TELESCOPE_UP_OUTPUT;
    }

    _climber.SetTelescopeArmOutput(telescopeOutput);
}

void ManualTelescopeCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _climber.SetTelescopeArmOutput(RESET_OUTPUT);
}

bool ManualTelescopeCommand::IsFinished()
{
    return _turretGood && !_humanControl.GetDesired(Buttons::kTeleArmUpButton) &&
           !_humanControl.GetDesired(Buttons::kTeleArmDownButton);
}