#include "subsystems/Turret.h"

#include <iostream>

#include "microsystems/SensorBoard.h"

Turret::Turret(SensorBoard& sensorControl) : _sensorControl{sensorControl}
{
    printf("initialized turret");
    ConfigTurretMotor();
    ConfigureShuffleboard();
}

void Turret::ConfigTurretMotor()
{
    _turretMotor.SetNeutralMode(NeutralMode::Brake);
    _turretMotor.SetInverted(true);
    _turretMotor.ConfigFactoryDefault();

    /* Configure Sensor Source for Primary PID */
    _turretMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, 10);
    _turretMotor.SetSensorPhase(true);
    _turretMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_13_Base_PIDF0, 10, 10);
    _turretMotor.SetStatusFramePeriod(StatusFrameEnhanced::Status_10_MotionMagic, 10, 10);
    _turretMotor.ConfigNominalOutputForward(0, 10);
    _turretMotor.ConfigNominalOutputReverse(0, 10);
    _turretMotor.ConfigPeakOutputForward(1.0, 10);
    _turretMotor.ConfigPeakOutputReverse(-1.0, 10);

    _turretMotor.SelectProfileSlot(0, 0);
    _turretMotor.Config_kF(0, _lowerLeftPotAngleEntry.GetDouble(0.0), 10);   // low left 0.106
    _turretMotor.Config_kP(0, _upperLeftPotAngleEntry.GetDouble(0.0), 10);   // up left
    _turretMotor.Config_kI(0, _lowerRightPotAngleEntry.GetDouble(0.0), 10);  // low right
    _turretMotor.Config_kD(0, _upperRightPotAngleEntry.GetDouble(0.0), 10);  // up right

    // left offset, 3605
    _turretMotor.ConfigMotionCruiseVelocity(_leftOffsetPotAngleEntry.GetDouble(0.0), 10);
    // right offset 3605
    _turretMotor.ConfigMotionAcceleration(_rightOffsetPotAngleEntry.GetDouble(0.0), 10);

    // zero sensor
    _turretMotor.SetSelectedSensorPosition(
        (_turretPotentiometer.Get() - POTENTIOMETER_MIN) * POT_TO_ENC, 0, 10);
    std::cout << "setting encoder to" << _turretMotor.GetSelectedSensorPosition()
              << " potentiometer " << _turretPotentiometer.Get() << std::endl;
    _turretMotor.ConfigAllowableClosedloopError(0, ENC_TOLERANCE, 10);
    _turretMotor.ConfigForwardSoftLimitEnable(true, 10);
    _turretMotor.ConfigReverseSoftLimitEnable(true, 10);
    _turretMotor.ConfigForwardSoftLimitThreshold(ENCODER_MAX, 10);
    _turretMotor.ConfigReverseSoftLimitThreshold(ENCODER_MIN, 10);
}

void Turret::Periodic()
{
    // Run vision controller periodic
    _visionController.Update();

    _currTurretEnc   = _turretMotor.GetSelectedSensorPosition();
    _turretIsAligned = GetAligned();
    _isAtZeroAngle   = IsAtZeroAngle();
    _sensorControl.SetAtTurretClimbAngle(IsAtClimbAngle());

    UpdateShuffleboard();
}

void Turret::GoToSetpoint(double currTurretAngle, double desiredAngle)
{
    _turretMotor.ConfigMotionCruiseVelocity(7210, 10);  // left offset, 3605
    _turretMotor.ConfigMotionAcceleration(7210, 10);    // right offset 3605
    _turretMotor.Set(TalonSRXControlMode::MotionMagic,
                     std::clamp(desiredAngle, ENCODER_MIN, ENCODER_MAX));
    SetTurretErrorEntry(desiredAngle - currTurretAngle);
}

void Turret::SpinTurretMotor(double power)
{
    _turretMotor.Set(TalonSRXControlMode::PercentOutput, power);
}

bool Turret::IsAtClimbAngle()
{
    return fabs(GetTurretAngle() - CLIMB_TURRET_ANGLE) < ANGLE_TO_POT * 8;
}

bool Turret::IsAtZeroAngle()
{
    return fabs(GetTurretAngle() - ZERO_TURRET_ANGLE) < ANGLE_TO_POT * 3;
}

double Turret::GetTurretAngle() { return _turretPotentiometer.Get(); }

double Turret::GetTurretAngleEnc() { return _turretMotor.GetSelectedSensorPosition(); }

bool Turret::GetAligned() { return fabs(GetTargetAngle()) < 1.5; }

void Turret::GoToScanSetpoint(double currTurretAngle, double desiredAngle)
{
    // left offset, 3605
    _turretMotor.ConfigMotionCruiseVelocity(2000, 10);
    // right offset 3605
    _turretMotor.ConfigMotionAcceleration(_rightOffsetPotAngleEntry.GetDouble(0.0), 10);
    _turretMotor.Set(TalonSRXControlMode::MotionMagic,
                     std::clamp(desiredAngle, ENCODER_MIN, ENCODER_MAX));
    SetTurretErrorEntry(desiredAngle - currTurretAngle);
}

void Turret::ConfigureShuffleboard()
{
    auto& superstructureTab = _sensorControl.GetSuperstructureTab();
    _turretEncoderEntry     = superstructureTab.Add("turret encoder", 0.0).GetEntry();
    // right is climb, left is intake
    _lowerLeftPotAngleEntry   = superstructureTab.Add("turret kF", 0.141).GetEntry();
    _upperLeftPotAngleEntry   = superstructureTab.Add("turretkP", 0.14).GetEntry();
    _lowerRightPotAngleEntry  = superstructureTab.Add("turret kI", 0.0).GetEntry();
    _upperRightPotAngleEntry  = superstructureTab.Add("turret kD", 0.0).GetEntry();
    _leftOffsetPotAngleEntry  = superstructureTab.Add("max vel", 7210.0).GetEntry();
    _rightOffsetPotAngleEntry = superstructureTab.Add("max accel", 7210.0).GetEntry();
    _turretEncError           = superstructureTab.Add("turret error", 0.0).GetEntry();
    _limelightDistanceEntry   = superstructureTab.Add("Limelight distance", 0.0).GetEntry();

    _isAlignedEntry = _sensorControl.GetDriverTab()
                          .Add("TURRET ALIGNED!", false)
                          .WithSize(3, 1)
                          .WithPosition(0, 3)
                          .GetEntry();
    _isTurretZeroEntry =
        _sensorControl.GetDriverTab().Add("TURRET ZEROED", false).WithPosition(1, 2).GetEntry();
    _climbTurretEntry =
        _sensorControl.GetDriverTab().Add("CLIMB TURRET", false).WithPosition(2, 2).GetEntry();
}

void Turret::UpdateShuffleboard()
{
    _climbTurretEntry.SetBoolean(_climbTurretScheduled);
    _isAlignedEntry.SetBoolean(GetTurretAligned());
    _isTurretZeroEntry.SetBoolean(_zeroTurretScheduled);
    _turretEncoderEntry.SetDouble(_currTurretEnc);
    _isAlignedEntry.SetDouble(_turretIsAligned);
    _limelightDistanceEntry.SetDouble(GetTargetDistance());
}

bool Turret::GetTurretAligned() { return _turretIsAligned; }

bool Turret::GetZeroTurret() { return _zeroTurretScheduled; }

void Turret::SetZeroTurret(bool zeroTurretScheduled) { _zeroTurretScheduled = zeroTurretScheduled; }

void Turret::SetTurretErrorEntry(double encoderError) { _turretEncError.SetDouble(encoderError); }

bool Turret::GetClimbTurret() { return _climbTurretScheduled; }

void Turret::SetClimbTurret(bool climbTurretScheduled)
{
    _climbTurretScheduled = climbTurretScheduled;
}

void Turret::SetAutoFreezeTurret(double desired) { _autoFreezeTurretDesired = desired; }
bool Turret::GetAutoFreezeTurret() { return _autoFreezeTurretDesired; }

bool Turret::IsAtTurretZeroAngle() { return _isAtZeroAngle; }

double Turret::GetTargetAngle() { return _visionController.GetPhotonAngle().value(); }
double Turret::GetTargetDistance()
{
    return std::clamp(_visionController.GetPhotonDistance().value(), 0.0, 18.0);
}
