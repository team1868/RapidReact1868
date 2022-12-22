#include "commands/ShootCommand.h"

#include "microsystems/ControlBoard.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Indexer.h"
#include "subsystems/Shooter.h"
#include "subsystems/Turret.h"
#include "utils/INDEXER_CONSTANTS.h"
#include "utils/SHOOTER_CONSTANTS.h"

ShootCommand::ShootCommand(Shooter& shooter,
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
    AddRequirements({&_shooter, &_turret, &_indexer});
}

void ShootCommand::Initialize() { _sensorControl.SetDesiredColor(1); }

void ShootCommand::Execute()
{
    _usingFarShotPrep = _humanControl.GetDesired(Buttons::kFarShootPrepButton);

    // as long as shoot button is not pressed, update desired velocity and hood height
    if (!_humanControl.GetDesired(Buttons::kShootButton)) {
        if (_usingFarShotPrep) {
            // for launch pad
            _desiredVelocity = FAR_SHOOT_VEL;
            _shooter.SetHoodHeight(FAR_HOOD_HEIGHT);
        } else {
            _desiredVelocity = _shooter.CalculateFlywheelVelocityDesired();
            _shooter.SetHoodHeight(_shooter.CalculateHoodHeight());
        }
    }

    _shooter.SetFlywheelRPM(_desiredVelocity);

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

void ShootCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _indexer.SetElevatorPower(RESET_OUTPUT);
    _indexer.SetIndexRollersPower(RESET_OUTPUT);
}

bool ShootCommand::IsFinished()
{
    return !_usingFarShotPrep && !_humanControl.GetDesired(Buttons::kShootButton);
}