#include "commands/PivotCommand.h"

#include <frc2/command/PIDCommand.h>

#include "microsystems/SensorBoard.h"
#include "subsystems/DriveTrain.h"

// default turret command
PivotCommand::PivotCommand(DriveTrain& drivetrain, SensorBoard& sensorControl, double desiredAngle)
    : _drivetrain{drivetrain}, _sensorControl{sensorControl}
{
    _pivotPID.SetTolerance(_turnTolerance);
    _pivotPID.EnableContinuousInput(-180.0, 180.0);
    _pivotPID.SetSetpoint(desiredAngle);

    AddRequirements({&_drivetrain});
}

void PivotCommand::Initialize()
{
    _pivotPID.SetPID(_drivetrain.GetPivotP(), _drivetrain.GetPivotI(), _drivetrain.GetPivotD());
    _pivotCommandStartTime = _sensorControl.GetCurTime();
}

void PivotCommand::Execute()
{
    double outputPower =
        std::clamp<double>(_pivotPID.Calculate(_sensorControl.GetNavXYaw()), -0.7, 0.7);
    _drivetrain.SetDriveValues(DriveTrain::kLeftWheels, -outputPower, true);
    _drivetrain.SetDriveValues(DriveTrain::kRightWheels, outputPower, true);
}

bool PivotCommand::IsFinished()
{
    return _pivotPID.AtSetpoint() ||
           _sensorControl.GetCurTime() - _pivotCommandStartTime > _pivotTimeoutSec;
}

void PivotCommand::End(bool interrupted)
{
    _drivetrain.SetDriveValues(DriveTrain::kAllWheels, 0.0, true);
}
