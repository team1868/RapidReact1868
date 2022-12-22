#include "RobotContainer.h"

#include <frc2/command/CommandScheduler.h>
#include <iostream>

#include "commands/GroundOuttakeCommand.h"
#include "commands/IntakeCommand.h"
#include "commands/ManualBackwardsCommand.h"
#include "commands/PivotCommand.h"
#include "commands/ShortShotCommand.h"
#include "commands/defaultcommands/DefaultClimbCommand.h"
#include "commands/defaultcommands/DefaultDriveCommand.h"
#include "commands/defaultcommands/DefaultShootPrepCommand.h"

RobotContainer::RobotContainer()
{
    printf("init robot container");

    frc::Shuffleboard::GetTab("Drive Team Control")
        .Add(_autoChooser)
        .WithWidget(frc::BuiltInWidgets::kComboBoxChooser)
        .WithSize(3, 1);

    // _autoChooser.AddOption("Drive Forward", &_driveForwardAuto);
    // _autoChooser.AddOption("Drive Forward & Back", &_testDriveAuto);
    // _autoChooser.SetDefaultOption("Simple Auto", &_testSuperAuto);
    _autoChooser.AddOption("Two Ball RED", &_twoBallAuto);
    _autoChooser.AddOption("TWO BALL DEFENSE DARK BLUE", &_twoBallDefense);
    // _autoChooser.AddOption("Three Ball Roll It Natalie YELLOW", &threeBallAuto_);
    _autoChooser.AddOption("Four Ball PURPLE", &_fourBallAuto);
    // _autoChooser.AddOption("Three Ball KMM", &threeKMMAuto_);
    // _autoChooser.AddOption("Trans Five Ball", &_fiveBallTrans);

    ConfigureDefaultCommands();
    ConfigureButtonBindings();

    printf("end of robot container constructor\n");
}

void RobotContainer::ConfigureDefaultCommands()
{
    // set default commands
    _drivetrain.SetDefaultCommand(DefaultDriveCommand(_drivetrain, _humanControl));
    _indexer.SetDefaultCommand(_defaultIndexCommand);
    _shooter.SetDefaultCommand(DefaultShootPrepCommand(_shooter, _indexer, _sensorControl));
    _turret.SetDefaultCommand(_defaultTurretNewCommand);
    _climber.SetDefaultCommand(DefaultClimbCommand(_climber));
}

void RobotContainer::ConfigureButtonBindings()
{
    // intaking
    _humanControl.IntakeRightButton.WhenPressed(
        IntakeCommand{_intake, _sensorControl, _humanControl}, false);

    // run everything backwards (undo!)
    _humanControl.OuttakeLeftButton.WhenPressed(
        ManualBackwardsCommand{_intake, _indexer, _sensorControl, _humanControl}, false);

    _humanControl.CloseShootPrepButton.WhenPressed(
        ShortShotCommand{_shooter, _turret, _indexer, _sensorControl, _humanControl}, true);
    _humanControl.FarShootPrepButton.WhenPressed(_shootCommand, true);
    _humanControl.AbortClimbButton.WhenPressed(_abortClimbCommand, true);

    _humanControl.GearShiftButton
        .WhenPressed(frc2::InstantCommand{[&] { _drivetrain.SetHighGear(); }})
        .WhenReleased(frc2::InstantCommand{[&] { _drivetrain.SetLowGear(); }});

    _humanControl.DefenseButton
        .WhenPressed(frc2::InstantCommand{[&] { _climber.DisengagePivotHook(); }})
        .WhenReleased(frc2::InstantCommand{[&] { _climber.EngagePivotHook(); }});

    (_humanControl.PivotArmInButton || _humanControl.PivotArmOutButton)
        .WhenActive(frc2::InstantCommand{[&] {
            _shooter.LowerHood();
            _sensorControl.SetBlockerDesired(false);
            _turret.SetClimbTurret(true);

            frc2::CommandScheduler::GetInstance().Schedule(true, &_climbTurretCommand);
            frc2::CommandScheduler::GetInstance().Schedule(true, &_manualPivotArmCommand);
        }});

    _humanControl.TraversalButton.WhenPressed(frc2::InstantCommand{[&] {
        _shooter.LowerHood();
        _sensorControl.SetBlockerDesired(false);
        _turret.SetClimbTurret(true);

        frc2::CommandScheduler::GetInstance().Schedule(true, &_climbTurretCommand);
        // traversal climb
        frc2::CommandScheduler::GetInstance().Schedule(true, &_autoClimbCommand);
    }});

    // blocker
    _humanControl.BlockerButton.WhenPressed(frc2::InstantCommand{[&] {
        if (!_sensorControl.IsAutoClimbing() && !_sensorControl.IsBlockerDesired()) {
            frc2::CommandScheduler::GetInstance().Schedule(true, &_zeroTurretCommand);
            _turret.SetZeroTurret(true);
        }
    }});

    // manual telescoping arm
    (_humanControl.TeleArmDownButton || _humanControl.TeleArmUpButton)
        .WhenActive(frc2::InstantCommand{[&] {
            _shooter.LowerHood();
            _sensorControl.SetBlockerDesired(false);

            frc2::CommandScheduler::GetInstance().Schedule(true, &_climbTurretCommand);
            _turret.SetClimbTurret(true);

            frc2::CommandScheduler::GetInstance().Schedule(true, &_manualTelescopeCommand);
        }});

    _humanControl.ZeroTurretButton.WhenPressed(frc2::InstantCommand{[&] {
        if (!_turret.GetZeroTurret()) {
            _turret.SetClimbTurret(false);
            frc2::CommandScheduler::GetInstance().Schedule(true, &_zeroTurretCommand);
        }
    }});

    _humanControl.ClimbTurretButton.WhenPressed(frc2::InstantCommand{[&] {
        if (!_turret.GetClimbTurret()) {
            _turret.SetZeroTurret(false);
            frc2::CommandScheduler::GetInstance().Schedule(true, &_climbTurretCommand);
        }
    }});

    // high climb
    _humanControl.HighClimbButton.WhenPressed(frc2::InstantCommand{[&] {
        if (!_sensorControl.IsAutoClimbing()) {
            _shooter.LowerHood();
            _sensorControl.SetBlockerDesired(false);

            frc2::CommandScheduler::GetInstance().Schedule(true, &_climbTurretCommand);
            _turret.SetClimbTurret(true);

            frc2::CommandScheduler::GetInstance().Schedule(true, &_manualClimbCommand);
        }
    }});

    // start climbing sequence after all the way up
    _humanControl.UpButton.WhenPressed(frc2::InstantCommand{[&] {
        _shooter.LowerHood();
        _sensorControl.SetBlockerDesired(false);

        frc2::CommandScheduler::GetInstance().Schedule(true, &_climbTurretCommand);
        _turret.SetClimbTurret(true);

        frc2::CommandScheduler::GetInstance().Schedule(true, &_raiseTeleArmCommand);
    }});

    _humanControl.GroundOuttakeButton.WhileHeld(
        GroundOuttakeCommand{_intake, _indexer, _humanControl});
    _humanControl.ShootButton.WhenPressed(frc2::InstantCommand{[&] {
        if (!_humanControl.GetDesired(Buttons::kCloseShootPrepButton)) {
            frc2::CommandScheduler::GetInstance().Schedule(true, &_shootCommand);
        }
    }});
}

void RobotContainer::ConfigFMSData()
{
    _sensorControl.SetAlliance(frc::DriverStation::GetAlliance());
}

void RobotContainer::ConfigAuto() { _sensorControl.SetAuto(); }

void RobotContainer::ConfigTeleop() { _sensorControl.SetTeleop(); }

void RobotContainer::Reinit()
{
    _climber.Reinit(_humanControl.GetDesired(Buttons::kDefenseButton));
}

void RobotContainer::UpdateSensorsDisabled() { UpdateSensors(); }
void RobotContainer::UpdateSensors()
{
    _sensorControl.Update();
    _leds.UpdateLED(_sensorControl.GetDesiredColor(),
                    _sensorControl.IsAutoClimbing(),
                    _sensorControl.GetTurretLEDOverride(),
                    _sensorControl.GetIndexLEDDesired());
}

void RobotContainer::UpdateShuffleboard() { _sensorControl.UpdateShuffleboard(); }

void RobotContainer::ResetDriveEncoders() { _drivetrain.ResetEncoders(); }

frc2::Command* RobotContainer::GetAutonomousCommand()
{
    if (_autoChooser.GetSelected() == &_twoBallAuto) {
        _sensorControl.SetDesiredColor(1);
        // } else if (_autoChooser.GetSelected() == &_threeBallAuto) {
        //     _sensorControl.SetDesiredColor(2);
    } else if (_autoChooser.GetSelected() == &_fourBallAuto) {
        _sensorControl.SetDesiredColor(3);
    } else if (_autoChooser.GetSelected() == &_twoBallDefense) {
        _sensorControl.SetDesiredColor(5);
    } else {
        _sensorControl.SetDesiredColor(0);
    }
    return _autoChooser.GetSelected();
}

SensorBoard& RobotContainer::GetSensorBoard() { return _sensorControl; }
ControlBoard& RobotContainer::GetHumanControl() { return _humanControl; }
DriveTrain& RobotContainer::GetDriveTrain() { return _drivetrain; }
Turret& RobotContainer::GetTurret() { return _turret; }

void RobotContainer::ResetClimbCommands()
{
    frc2::CommandScheduler::GetInstance().Schedule(true, &_abortClimbCommand);
}
