#include "commands/defaultcommands/DefaultClimbCommand.h"

#include "subsystems/Climber.h"

DefaultClimbCommand::DefaultClimbCommand(Climber& climber) : _climber{climber}
{
    AddRequirements({&climber});
}

void DefaultClimbCommand::Initialize()
{
    _climber.SetPivotArmOutput(RESET_OUTPUT);
    _climber.SetTelescopeArmOutput(RESET_OUTPUT);
}
