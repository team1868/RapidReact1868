#pragma once

#include <AHRS.h>
#include <frc/DriverStation.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>

#include "utils/PORTS.h"
#include "utils/SENSOR_CONSTANTS.h"

class SensorBoard {
   public:
    SensorBoard();

    void Update();
    void UpdateShuffleboard();

    // timer
    double GetTime();
    double GetCurTime();

    // for pivot/drive
    double GetNavXYaw();
    double GetNavXPitch();
    double GetNavXRoll();
    void ResetNavX();

    // shuffleboard
    frc::ShuffleboardTab& GetSetupTab();
    frc::ShuffleboardTab& GetSuperstructureTab();
    frc::ShuffleboardTab& GetDriverTab();

    void SetAuto();
    void SetTeleop();
    bool IsAuto();

    frc::Rotation2d GetRotation2d();

    void SetAlliance(frc::DriverStation::Alliance alliance);
    frc::DriverStation::Alliance GetAlliance();

    void SetDesiredColor(int desired);
    int GetDesiredColor();

    void SetTurretLEDOverride(bool LEDoverride);  // false when aligned
    bool GetTurretLEDOverride();                  // false when aligned

    void SetIndexLEDDesired(int indexLEDdesired);
    bool GetIndexLEDDesired();

    void SetCurrentlyClimbing(bool isClimbing);
    bool IsCurrentlyClimbing();

    void SetAtTurretClimbAngle(bool isAtAngle);
    bool IsAtTurretClimbAngle();

    bool IsIntaking();
    void SetIntaking(bool val);

    bool IsPosSwing();
    bool SwingDirJustChanged();

    bool IsAutoClimbing();
    void SetAutoClimbing(bool val);

    bool IsBlockerDesired();
    void SetBlockerDesired(bool desired);

   private:
    // time utilities
    frc::Timer _timer{};
    double _curTime;

    // Shared sensors and shuffleboard resources

    AHRS _navX{frc::SPI::kMXP, NAVX_SPEED};
    bool _curSwing = false, _prevSwing = false;
    double _curPitch;
    frc::ShuffleboardTab &_setupTab, &_superstructureTab, &_driverTab;
    frc::DriverStation::Alliance _alliance;

    // Synchronization locks, values, and variables
    bool _isAuto              = false;
    bool _isCurrentlyClimbing = false;
    bool _isAtClimbAngle      = false;
    bool _isCurIntaking       = false;
    bool _curAutoClimbing     = false;
    bool _blockerDesired      = false;

    // LED values
    bool _LEDoverride = false;
    int _desiredColor = 0, _indexLEDdesired = -1;
};
