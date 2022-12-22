#include "commands/turretcommands/ZeroTurretCommand.h"

#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"
#include "subsystems/Turret.h"

// default turret command
ZeroTurretCommand::ZeroTurretCommand(Turret& turret, Climber& climber, SensorBoard& sensorControl)
    : _turret{turret}, _climber{climber}, _sensorControl{sensorControl}
{
    AddRequirements({&_turret});
}

void ZeroTurretCommand::Initialize()
{
    _turret.SpinTurretMotor(0.0);
    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3);  // on
    _sensorControl.SetDesiredColor(5);
}

void ZeroTurretCommand::Execute()
{
    if (!_climber.GetTeleHardStopLimitSwitch() && _turret.GetZeroTurret()) {
        _turret.GoToSetpoint(_turret.GetTurretAngleEnc(),
                             (POTENTIOMETER_OFFSET - POTENTIOMETER_MIN) * POT_TO_ENC);
    }
}

void ZeroTurretCommand::End(bool interrupted)
{
    _sensorControl.SetDesiredColor(0);
    _turret.SpinTurretMotor(0.0);
}

bool ZeroTurretCommand::IsFinished() { return !_turret.GetZeroTurret(); }