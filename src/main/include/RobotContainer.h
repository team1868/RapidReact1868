#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/InstantCommand.h>

#include "commands/ShootCommand.h"
#include "commands/autocommands/AutoSequences.h"
#include "commands/climbcommands/AbortClimbCommand.h"
#include "commands/climbcommands/AutoClimbCommand.h"
#include "commands/climbcommands/ManualClimbCommand.h"
#include "commands/climbcommands/ManualPivotArmCommand.h"
#include "commands/climbcommands/ManualTelescopeCommand.h"
#include "commands/climbcommands/RaiseTeleArmCommand.h"
#include "commands/defaultcommands/DefaultIndexCommand.h"
#include "commands/turretcommands/ClimbTurretCommand.h"
#include "commands/turretcommands/DefaultTurretNewCommand.h"
#include "commands/turretcommands/ZeroTurretCommand.h"
#include "microsystems/ControlBoard.h"
#include "microsystems/LEDController.h"
#include "microsystems/SensorBoard.h"
#include "subsystems/Blocker.h"
#include "subsystems/Climber.h"
#include "subsystems/DriveTrain.h"
#include "subsystems/Indexer.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/Turret.h"

class RobotContainer {
   public:
    RobotContainer();

    frc2::Command* GetAutonomousCommand();
    void ConfigureButtonBindings();
    void ConfigureDefaultCommands();
    void UpdateSensors();
    void UpdateSensorsDisabled();
    void UpdateShuffleboard();
    void ResetDriveEncoders();
    void ConfigFMSData();
    void ConfigAuto();
    void ConfigTeleop();
    void Reinit();
    void ResetClimbCommands();

    DriveTrain& GetDriveTrain();
    SensorBoard& GetSensorBoard();
    ControlBoard& GetHumanControl();
    Turret& GetTurret();

   private:
    // Subsystems
    SensorBoard _sensorControl{};
    ControlBoard _humanControl{};

    Intake _intake{};
    DriveTrain _drivetrain{_sensorControl};
    Indexer _indexer{};
    Turret _turret{_sensorControl};
    Shooter _shooter{_turret, _sensorControl};
    Blocker _blocker{_turret, _sensorControl};
    Climber _climber{_sensorControl};
    LEDController _leds{};

    // Default Commands
    DefaultTurretNewCommand _defaultTurretNewCommand{_turret, _climber, _sensorControl};
    DefaultIndexCommand _defaultIndexCommand{_indexer, _sensorControl, _humanControl};

    // Commands
    ShootCommand _shootCommand{_shooter, _turret, _indexer, _sensorControl, _humanControl};

    // turret commands
    ZeroTurretCommand _zeroTurretCommand{_turret, _climber, _sensorControl};
    ClimbTurretCommand _climbTurretCommand{_turret, _shooter, _sensorControl};

    // climb commands
    AbortClimbCommand _abortClimbCommand{_climber, _sensorControl};
    ManualPivotArmCommand _manualPivotArmCommand{_climber, _sensorControl, _humanControl};
    ManualTelescopeCommand _manualTelescopeCommand{_climber, _sensorControl, _humanControl};

    RaiseTeleArmCommand _raiseTeleArmCommand{_climber, _sensorControl};  // 1
    // PullUpCommand _pullUpCommand{_climber, _sensorControl};                       // 2
    // ReachBackCommand _reachBackCommand{_climber, _sensorControl};                 // 3
    // RaiseToNextRungCommand _raiseToNextRungCommand_{_climber, _sensorControl};    // 4
    // CollapseCommand _collapseCommand{_climber, _sensorControl};                   // 5
    // EndPullUpCommand _endPullUpCommand{_climber, _sensorControl, _humanControl};  // 6
    // HaltPullUpCommand _haltPullUpCommand{_climber, _sensorControl};

    AutoClimbCommand _autoClimbCommand{_climber, _sensorControl, _humanControl};
    ManualClimbCommand _manualClimbCommand{_climber, _sensorControl, _humanControl};

    // Auto commands
    // DriveStraightCommand _driveForwardAuto{_drivetrain, 0.4, 5};
    // TestDriveAuto _testDriveAuto{_drivetrain};
    // TestSuperAuto _testSuperAuto{_shooter, _indexer, _intake, _sensorControl};

    frc2::InstantCommand _stopDrivetrainCommand{[&] { _drivetrain.ZeroDriveVolts(); }, {}};

    TwoBaller _twoBallAuto{_drivetrain,
                           _intake,
                           _shooter,
                           _sensorControl,
                           _turret,
                           _indexer,
                           _stopDrivetrainCommand,
                           _defaultIndexCommand};

    TwoBallDefense _twoBallDefense{_drivetrain,
                                   _intake,
                                   _shooter,
                                   _sensorControl,
                                   _turret,
                                   _indexer,
                                   _stopDrivetrainCommand,
                                   _defaultIndexCommand};

    // ThreeBallHP _threeBallAuto{_drivetrain,
    //                            _intake,
    //                            _shooter,
    //                            _sensorControl,
    //                            _turret,
    //                            _indexer,
    //                            _defaultIndexCommand,
    //                            _defaultTurretNewCommand};

    FourBall _fourBallAuto{_drivetrain,
                           _intake,
                           _shooter,
                           _sensorControl,
                           _turret,
                           _indexer,
                           _stopDrivetrainCommand,
                           _defaultIndexCommand,
                           _defaultTurretNewCommand};

    // ThreeBallKMM _threeKMMAuto{_drivetrain,
    //                            _intake,
    //                            _shooter,
    //                            _sensorControl,
    //                            _turret,
    //                            _indexer,
    //                            _stopDrivetrainCommand};

    // FiveBallTrans _fiveBallTrans{_drivetrain,
    //                              _intake,
    //                              _shooter,
    //                              _sensorControl,
    //                              _turret,
    //                              _indexer,
    //                              _stopDrivetrainCommand,
    //                              _defaultIndexCommand,
    //                              _defaultTurretNewCommand};

    // The chooser for the autonomous routines
    frc::SendableChooser<frc2::Command*> _autoChooser{};
};
