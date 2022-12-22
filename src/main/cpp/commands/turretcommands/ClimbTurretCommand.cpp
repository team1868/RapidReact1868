#include "commands/turretcommands/ClimbTurretCommand.h"

#include "microsystems/SensorBoard.h"
#include "subsystems/Shooter.h"
#include "subsystems/Turret.h"

// default turret command
ClimbTurretCommand::ClimbTurretCommand(Turret& turret, Shooter& shooter, SensorBoard& sensorControl)
    : _turret{turret}, _shooter{shooter}, _sensorControl{sensorControl}
{
    AddRequirements({&_turret, &_shooter});
}

void ClimbTurretCommand::Initialize()
{
    _shooter.LowerHood();
    _shooter.SetFlywheelRPM(0.0);

    _turret.SpinTurretMotor(0.0);
    nt::NetworkTableInstance::GetDefault()
        .GetTable("limelight")
        ->PutNumber("ledMode", 1);  // led off
    _sensorControl.SetDesiredColor(7);
}

void ClimbTurretCommand::Execute()
{
    // TODO future cleanup, make Turret subsystem manage it's own current turret angle
    _turret.GoToSetpoint(_turret.GetTurretAngleEnc(),
                         (CLIMB_TURRET_ANGLE - POTENTIOMETER_MAX) * POT_TO_ENC);
}

void ClimbTurretCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _turret.SpinTurretMotor(0.0);
}

bool ClimbTurretCommand::IsFinished() { return !_turret.GetClimbTurret(); }