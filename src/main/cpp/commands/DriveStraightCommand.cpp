#include "commands/DriveStraightCommand.h"

#include "subsystems/DriveTrain.h"

DriveStraightCommand::DriveStraightCommand(DriveTrain& drivetrain, double speed, double distance)
    : _drivetrain{drivetrain}, _speed{speed}, _distance{distance}
{
    AddRequirements({&drivetrain});
}

void DriveStraightCommand::Initialize() { _drivetrain.ResetEncoders(); }

void DriveStraightCommand::Execute()
{
    _drivetrain.ArcadeDrive(_speed, 0);
    _drivetrain.UpdateEncoderValues();
}

void DriveStraightCommand::End(bool interrupted) { _drivetrain.ArcadeDrive(0, 0); }

bool DriveStraightCommand::IsFinished()
{
    return std::abs(_drivetrain.GetAverageEncoderDistance()) >= _distance;
}
