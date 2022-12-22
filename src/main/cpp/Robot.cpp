#include "Robot.h"

#include <cameraserver/CameraServer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc2/command/CommandScheduler.h>
#include <iostream>

#include "microsystems/Logger.h"

void Robot::RobotInit()
{
    _camera1 = frc::CameraServer::StartAutomaticCapture();  // intake camera!
    _camera1.SetFPS(20);
    _camera1.SetResolution(160, 120);  //(160, 120);//0.13 second latency
    _camera1.SetVideoMode(cs::VideoMode::kMJPEG, 160, 120, 20);
    _mjpegServer.SetSource(_camera1);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
    frc2::CommandScheduler::GetInstance().Run();
    _container.UpdateSensors();
    _container.UpdateShuffleboard();
    // we know this is causing command scheduler overrun errors
    Logger::LogState(
        _container.GetDriveTrain(), _container.GetSensorBoard(), _container.GetHumanControl());
}

/**
 * This function is called once each time the robot enters Disabled mode. You can use it to reset
 * any subsystem information you want to clear when the robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic()
{
    _container.UpdateSensorsDisabled();
    _autonomousCommand = _container.GetAutonomousCommand();
}

/**
 * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
 */
void Robot::AutonomousInit()
{
    frc::LiveWindow::DisableAllTelemetry();
    _container.ConfigFMSData();
    _container.ResetDriveEncoders();
    _container.Reinit();
    _container.GetSensorBoard().ResetNavX();
    _container.GetDriveTrain().ResetOdometry(frc::Pose2d());

    _container.ConfigAuto();
    _autonomousCommand = _container.GetAutonomousCommand();

    _container.GetSensorBoard().ResetNavX();

    if (_autonomousCommand != nullptr) { _autonomousCommand->Schedule(); }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
    frc::LiveWindow::DisableAllTelemetry();
    _container.GetTurret().SetAutoFreezeTurret(false);  // no more freezing bc no more auto
    _container.ResetClimbCommands();
    _container.ConfigFMSData();
    _container.GetSensorBoard().ResetNavX();
    _container.GetDriveTrain().ResetOdometry(frc::Pose2d());
    _container.Reinit();

    // This makes sure that the autonomous stops running when teleop starts running. If you want the
    // autonomous to continue until interrupted by another command,  remove this line or comment it
    // out.
    _container.ResetDriveEncoders();
    _container.ConfigTeleop();
    if (_autonomousCommand != nullptr) {
        _autonomousCommand->Cancel();
        _autonomousCommand = nullptr;
    }

    frc2::CommandScheduler::GetInstance().CancelAll();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
