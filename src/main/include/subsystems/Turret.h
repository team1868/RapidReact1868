#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/Encoder.h>
#include <frc2/command/SubsystemBase.h>

#include "microsystems/VisionController.h"
#include "utils/PORTS.h"
#include "utils/TURRET_CONSTANTS.h"

class SensorBoard;

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class Turret : public frc2::SubsystemBase {
   public:
    Turret(SensorBoard& sensorControl);

    void SpinTurretMotor(double power);

    double GetTurretAngle();
    bool GetAligned();
    void GoToSetpoint(double currTurretAngle, double desiredAngle);
    bool IsAtClimbAngle();
    bool IsAtZeroAngle();
    void Periodic() override;
    void GoToScanSetpoint(double currTurretAngle, double desiredAngle);
    double GetTurretAngleEnc();
    void ConfigTurretMotor();
    void SetTurretErrorEntry(double encoderError);
    bool GetTurretAligned();

    void SetClimbTurret(bool climbTurretScheduled);
    bool GetClimbTurret();

    void SetAutoFreezeTurret(double desired);
    bool GetAutoFreezeTurret();

    void SetZeroTurret(bool zeroTurretScheduled);
    bool GetZeroTurret();

    bool IsAtTurretZeroAngle();
    double GetTargetAngle();
    double GetTargetDistance();

   private:
    SensorBoard& _sensorControl;

    WPI_TalonSRX _turretMotor{TURRET_WRIST_MOTOR_ID};
    frc::AnalogInput _input{ANALOG_POTENTIOMETER_PORT};
    frc::AnalogPotentiometer _turretPotentiometer{&_input, 3600.0, 0.0};

    VisionController _visionController{};
    nt::NetworkTableEntry _limelightDistanceEntry;

    nt::NetworkTableEntry _turretEncoderEntry;
    nt::NetworkTableEntry _lowerLeftPotAngleEntry, _upperLeftPotAngleEntry;
    nt::NetworkTableEntry _lowerRightPotAngleEntry, _upperRightPotAngleEntry;
    nt::NetworkTableEntry _leftOffsetPotAngleEntry, _rightOffsetPotAngleEntry;
    nt::NetworkTableEntry _isTurretZeroEntry, _isAlignedEntry;
    nt::NetworkTableEntry _turretEncError;
    nt::NetworkTableEntry _climbTurretEntry;

    double _currTurretEnc;
    bool _turretIsAligned = false, _zeroTurretScheduled = false, _climbTurretScheduled = false;
    bool _autoFreezeTurretDesired = false;
    bool _isAtZeroAngle           = false;

    void ConfigureShuffleboard();
    void UpdateShuffleboard();
};