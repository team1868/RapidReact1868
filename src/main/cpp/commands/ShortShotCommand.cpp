#include "commands/ShortShotCommand.h"

#include "microsystems/ControlBoard.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Indexer.h"
#include "subsystems/Shooter.h"
#include "subsystems/Turret.h"
#include "utils/INDEXER_CONSTANTS.h"

ShortShotCommand::ShortShotCommand(Shooter& shooter,
                                   Turret& turret,
                                   Indexer& indexer,
                                   SensorBoard& sensorControl,
                                   ControlBoard& humanControl)
    : _shooter{shooter},
      _turret{turret},
      _indexer{indexer},
      _sensorControl{sensorControl},
      _humanControl{humanControl}
{
    AddRequirements({&shooter, &turret, &indexer});
}

void ShortShotCommand::Initialize()
{
    _shooter.SetHoodHeight(_shooter.GetDesiredHood());
    _desiredVelocity = _shooter.GetDesiredRPM();
    _shooter.SetFlywheelRPM(_desiredVelocity);
    _sensorControl.SetDesiredColor(1);
}

void ShortShotCommand::Execute()
{
    if (_shooter.IsFlywheelAtSpeed(_desiredVelocity)) {
        _sensorControl.SetDesiredColor(2);
        if (_humanControl.GetDesired(Buttons::kShootButton)) {
            _indexer.SetElevatorPower(ELEVATOR_OUTPUT);
            _indexer.SetIndexRollersPower(INDEX_ROLLERS_OUTPUT);
        }
    } else {
        _sensorControl.SetDesiredColor(1);
        _indexer.SetElevatorPower(RESET_OUTPUT);
        _indexer.SetIndexRollersPower(RESET_OUTPUT);
    }
}

void ShortShotCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _shooter.SetFlywheelRPM(RESET_OUTPUT);
    _indexer.SetElevatorPower(RESET_OUTPUT);
    _indexer.SetIndexRollersPower(RESET_OUTPUT);
}

bool ShortShotCommand::IsFinished()
{
    return !_humanControl.GetDesired(Buttons::kCloseShootPrepButton);
}