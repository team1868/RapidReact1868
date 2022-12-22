#include "subsystems/DriveTrain.h"

#include <ctre/phoenix/motorcontrol/FeedbackDevice.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/CANSparkMax.h>
#include <iostream>

#include "microsystems/SensorBoard.h"

DriveTrain::DriveTrain(SensorBoard& sensorControl)
    : _sensorControl{sensorControl}, _odometry{_sensorControl.GetRotation2d()}
{
    std::cout << "start of drive" << std::endl;

    // Motors
    _leftPrimary.SetNeutralMode(NeutralMode::Coast);
    _rightPrimary.SetNeutralMode(NeutralMode::Coast);
    _leftSecondary.SetNeutralMode(NeutralMode::Coast);
    _rightSecondary.SetNeutralMode(NeutralMode::Coast);
    _leftPrimary.SetSafetyEnabled(false);
    _rightPrimary.SetSafetyEnabled(false);
    _leftSecondary.SetSafetyEnabled(false);
    _rightSecondary.SetSafetyEnabled(false);

    // Sets motors to accept outputs in percent
    _leftPrimary.Set(ControlMode::PercentOutput, 0.0);
    _rightPrimary.Set(ControlMode::PercentOutput, 0.0);

    // set up motor groups
    _rightMotors.SetInverted(true);

    _leftPrimary.ConfigSelectedFeedbackSensor(
        ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, _kSlot, _timeout);
    _leftPrimary.SelectProfileSlot(_kSlot, _kPIDIdx);
    _leftPrimary.Config_kP(_kSlot, DriveConstants::kPDriveVel);
    _leftPrimary.Config_kI(_kSlot, 0.0);
    _leftPrimary.Config_kD(_kSlot, 0.0);

    _rightPrimary.ConfigSelectedFeedbackSensor(
        ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor, _kSlot, _timeout);
    _rightPrimary.SelectProfileSlot(_kSlot, _kPIDIdx);
    _rightPrimary.Config_kP(_kSlot, DriveConstants::kPDriveVel);
    _rightPrimary.Config_kI(_kSlot, 0.0);
    _rightPrimary.Config_kD(_kSlot, 0.0);

    // left secondary and right secondary motors follow primary motors
    _leftSecondary.Follow(_leftPrimary);
    _rightSecondary.Follow(_rightPrimary);

    ConfigureShuffleboard();
    std::cout << "end of drive" << std::endl;
}

frc::Pose2d DriveTrain::GetPose() { return _odometry.GetPose(); }

void DriveTrain::TankDriveVolts(units::volt_t left, units::volt_t right)
{
    _leftMotors.SetVoltage(left);
    _rightMotors.SetVoltage(right);
    _drive.Feed();
}

void DriveTrain::ZeroDriveVolts() { TankDriveVolts(0_V, 0_V); }

void DriveTrain::ResetOdometry(frc::Pose2d pose)
{
    ResetEncoders();
    _odometry.ResetPosition(pose, _sensorControl.GetRotation2d());
}

void DriveTrain::Periodic()
{
    UpdateEncoderValues();
    // m_field.SetRobotPose(_odometry.GetPose());
    _odometry.Update(_sensorControl.GetRotation2d(), GetLeftDistance(), GetRightDistance());
    UpdateShuffleboard();
}

void DriveTrain::ArcadeDrive(double thrust, double rotate)
{
    // adjusting sensitivity for turn
    thrust = GetDeadbandAdjustment(thrust);
    thrust = GetCubicAdjustment(thrust, _thrustSensitivity);
    rotate = GetDeadbandAdjustment(rotate);
    rotate = GetCubicAdjustment(rotate, _rotateSensitivity);

    _drive.ArcadeDrive(thrust, rotate);
}

double DriveTrain::GetDeadbandAdjustment(double value)
{
    // if it's lower than the deadband, the robot should not move
    if (fabs(value) < DEADBAND_MAX) { return 0.0; }
    return (value + (value > DEADBAND_MAX ? -DEADBAND_MAX : DEADBAND_MAX)) / (1 - DEADBAND_MAX);
}

void DriveTrain::MaxSpeedAdjustment(double& leftvalue, double& rightvalue)
{
    if (leftvalue > 1.0) {
        rightvalue /= leftvalue;
        leftvalue = 1.0;
    } else if (leftvalue < -1.0) {
        rightvalue /= -leftvalue;
        leftvalue = -1.0;
    }
    if (rightvalue > 1.0) {
        leftvalue /= rightvalue;
        rightvalue = 1.0;
    } else if (rightvalue < -1.0) {
        leftvalue /= -rightvalue;
        rightvalue = -1.0;
    }
}

double DriveTrain::GetCubicAdjustment(double value, double adjustmentConstant)
{
    return adjustmentConstant * std::pow(value, 3.0) + (1.0 - adjustmentConstant) * value;
}

void DriveTrain::ResetEncoders()
{
    // _leftDriveEncoder.Reset();
    // _rightDriveEncoder.Reset();

    // read curr encoder values and store as initial encoder values
    _initialLeftEncoderValue  = _leftPrimary.GetSelectedSensorPosition();
    _initialRightEncoderValue = _rightPrimary.GetSelectedSensorPosition();
}

double DriveTrain::GetAverageEncoderDistance()
{
    return (GetLeftDistance().value() + GetRightDistance().value()) / 2.0;
}

double DriveTrain::GetLeftEncoderValue()
{
    return _leftPrimary.GetSelectedSensorPosition() - _initialLeftEncoderValue;  // not inverted
}

double DriveTrain::GetRightEncoderValue()
{
    return -(_rightPrimary.GetSelectedSensorPosition() - _initialRightEncoderValue);
}

double DriveTrain::GetRawLeftEncoderValue() { return _leftPrimary.GetSelectedSensorPosition(); }

double DriveTrain::GetRawRightEncoderValue() { return _rightPrimary.GetSelectedSensorPosition(); }

units::meter_t DriveTrain::GetLeftDistance()
{
    return units::meter_t{GetLeftEncoderValue() / _isHighGear ? HGEAR_TICKS_PER_METER
                                                              : LGEAR_TICKS_PER_METER};
}

units::meter_t DriveTrain::GetRightDistance()
{
    return units::meter_t{GetRightEncoderValue() / _isHighGear ? HGEAR_TICKS_PER_METER
                                                               : LGEAR_TICKS_PER_METER};
}

void DriveTrain::UpdateEncoderValues()
{
    _lastLeftEncoderValue  = _currLeftEncoderValue;
    _lastRightEncoderValue = _currRightEncoderValue;
    _currLeftEncoderValue  = GetLeftEncoderValue();
    _currRightEncoderValue = GetRightEncoderValue();

    _leftDriveEncoderEntry.SetDouble(_currLeftEncoderValue);
    _rightDriveEncoderEntry.SetDouble(_currRightEncoderValue);
    _leftDriveDistanceEntry.SetDouble(GetLeftDistance().value());
    _rightDriveDistanceEntry.SetDouble(GetRightDistance().value());
}

frc::DifferentialDriveWheelSpeeds DriveTrain::GetWheelSpeeds()
{
    // units meter per second but sensor velocity reported per 100 ms
    return {units::meters_per_second_t(_leftPrimary.GetSelectedSensorVelocity() /
                                       HGEAR_TICKS_PER_METER * 10),
            units::meters_per_second_t(_rightPrimary.GetSelectedSensorVelocity() /
                                       HGEAR_TICKS_PER_METER * -10)};
}

void DriveTrain::SetHighGear()
{
    if (!_isHighGear) {
        _gearSolenoid.Set(true);
        _isHighGear = true;
    }
    ResetDriveEncoders();
}

void DriveTrain::SetLowGear()
{
    if (_isHighGear) {
        _gearSolenoid.Set(false);
        _isHighGear = false;
    }
    ResetDriveEncoders();
}

bool DriveTrain::IsHighGear() { return _isHighGear; }

void DriveTrain::ResetDriveEncoders()
{
    _initialLeftEncoderValue  = GetRawLeftEncoderValue();
    _initialRightEncoderValue = GetRawRightEncoderValue();
}

double DriveTrain::GetDriveStraightDistance() { return _dsDistEntry.GetDouble(0.0); }

void DriveTrain::SetDriveValues(DriveTrain::Wheels wheel, double value, bool isAutonomous)
{
    if (isAutonomous == false) {  // only do brownout if not in auto
        value = ModifyCurrent(LEFT_DRIVE_MOTOR_A_PDP_CHAN, value);
        // value = -value;
        // lol or just dont do this and flip the negatives below OR get right value above
    }

    switch (wheel) {
        case (kLeftWheels):  // set left
            _leftPrimary.Set(value);
            break;
        case (kRightWheels):  // set right
            _rightPrimary.Set(value);
            break;
        case (kAllWheels):  // set both
            _rightPrimary.Set(value);
            _leftPrimary.Set(value);
            break;
        default: printf("WARNING: Drive value not set in DriveTrain::SetDriveValues()");
    }
}

// returns pivot p
double DriveTrain::GetPivotP() { return _pPEntry.GetDouble(0.015); }

// returns pivot i
double DriveTrain::GetPivotI() { return _pIEntry.GetDouble(0.0); }

// returns pivot d
double DriveTrain::GetPivotD() { return _pDEntry.GetDouble(0.0); }

// brownout current adjustment
/**
 * Gets total current
 * @return a double, the total current
 */
double DriveTrain::GetTotalCurrent() { return _pdp.GetTotalCurrent(); }

/**
 * Gets current voltage
 * @return a double, the current voltage
 */
double DriveTrain::GetVoltage() { return _pdp.GetVoltage(); }

// updates RATIO
void DriveTrain::UpdateCurrent()
{
    _leftDriveACurrent  = _leftPrimary.GetSupplyCurrent();
    _leftDriveBCurrent  = _leftPrimary.GetSupplyCurrent();
    _rightDriveACurrent = _rightPrimary.GetSupplyCurrent();
    _rightDriveBCurrent = _rightPrimary.GetSupplyCurrent();

    if ((GetTotalCurrent() > _maxCurrentEntry.GetDouble(MAX_CURRENT_OUTPUT) ||
         GetVoltage() <= _minVoltEntry.GetDouble(MIN_BROWNOUT_VOLTAGE)) &&
        !_lastOver) {
        // was not over limit before
        // if (_ratioDrive*0.9 > MIN_RATIO_DRIVE_CURRENT){
        //  _ratioDrive *= 0.9; //first try to lower current
        // }
        _lastOver = true;
    } else if ((GetTotalCurrent() > _maxCurrentEntry.GetDouble(MAX_CURRENT_OUTPUT) ||
                GetVoltage() <= _minVoltEntry.GetDouble(MIN_BROWNOUT_VOLTAGE)) &&
               _lastOver) {
        // already was over limit
        // if (_ratioDrive > MIN_RATIO_DRIVE_CURRENT) {
        //     _ratioDrive *= _ratioDrive;             // decrease by a lot
        //     _ratioDrive = MIN_RATIO_DRIVE_CURRENT;  // just go back down, *0.9 wasnt enough!
        // }
        _lastOver = true;
    } else {
        // not over limit; move _ratioDrive towards 1
        // if (_ratioDrive + 0.001 < 1.0) {
        //     _ratioDrive *= 1.01;
        // } else if (_ratioDrive < 1.0) {
        //     _ratioDrive = 1.0;
        // }
        _lastOver = false;
    }
    _ratioDriveEntry.SetDouble(_ratioDrive);
}

/**
 * Gets current of a specific motor
 * @param channel an integer
 * @return current of the motor as a double
 */
double DriveTrain::GetCurrent(int channel)
{
    UpdateCurrent();
    switch (channel) {
        case RIGHT_DRIVE_MOTOR_A_PDP_CHAN: return _rightDriveACurrent;
        case RIGHT_DRIVE_MOTOR_B_PDP_CHAN: return _rightDriveBCurrent;
        case LEFT_DRIVE_MOTOR_A_PDP_CHAN: return _leftDriveACurrent;
        case LEFT_DRIVE_MOTOR_B_PDP_CHAN: return _leftDriveBCurrent;
        default: printf("WARNING: Current not recieved in DriveTrain::GetCurrent()\n"); return -1;
    }
}

void DriveTrain::SetLeftRightSpeeds(units::meters_per_second_t left_speed,
                                    units::meters_per_second_t right_speed)
{
    double gear = _isHighGear ? HGEAR_TICKS_PER_METER : LGEAR_TICKS_PER_METER;

    // Compute feedforward term.
    // TalonFX expects feedforward to range from -1 to 1, but SimpleMotorFeedforward.calculate()
    // returns a voltage. Divide by max voltage to get proper scaling.
    _left_ff  = _velocity_feedforward.Calculate(left_speed).value() / RATIO_BATTERY_VOLTAGE;
    _right_ff = _velocity_feedforward.Calculate(right_speed).value() / RATIO_BATTERY_VOLTAGE;

    // Call the TalonFX API to set the velocity setpoint, expects velocity in ticks per 100ms
    _leftPrimary.Set(TalonFXControlMode::Velocity,
                     left_speed.value() * gear * 0.1,
                     ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
                     _left_ff);
    _rightPrimary.Set(TalonFXControlMode::Velocity,
                      right_speed.value() * gear * 0.1,
                      ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
                      _right_ff);
}

/**
 * Checks motor current over power
 * @param channel an integer
 * @param power a double
 * @return a double with the ratio-ed down power by percent * _ratioDrive
 */
double DriveTrain::CheckMotorCurrentOver(int channel, double power)
{
    double motorCurrent = GetCurrent(channel);
    // current to individual motor is over
    if (motorCurrent > MAX_DRIVE_MOTOR_CURRENT) {
        power = MIN_RATIO_DRIVE_CURRENT * power * (MAX_DRIVE_MOTOR_CURRENT / motorCurrent);
        std::cout << "\nover current limit, power now" << power << std::endl;
    }
    return power;
}

/**
 * Modifies the current of a particular motor
 * @param channel an integer
 * @param value a double
 * @return a double, the new power
 */
double DriveTrain::ModifyCurrent(int channel, double value)
{
    // double power = value*_ratioDrive;
    double power                = value;
    double individualPowerRatio = power;
    double tempPowerRatio;

    switch (channel) {
        case LEFT_DRIVE_MOTOR_A_PDP_CHAN:
        case RIGHT_DRIVE_MOTOR_A_PDP_CHAN:
            tempPowerRatio = CheckMotorCurrentOver(RIGHT_DRIVE_MOTOR_A_PDP_CHAN, power);
            if (tempPowerRatio < individualPowerRatio) { individualPowerRatio = tempPowerRatio; }
            tempPowerRatio = CheckMotorCurrentOver(RIGHT_DRIVE_MOTOR_B_PDP_CHAN, power);
            if (tempPowerRatio < individualPowerRatio) { individualPowerRatio = tempPowerRatio; }
            tempPowerRatio = CheckMotorCurrentOver(LEFT_DRIVE_MOTOR_A_PDP_CHAN, power);
            if (tempPowerRatio < individualPowerRatio) { individualPowerRatio = tempPowerRatio; }
            tempPowerRatio = CheckMotorCurrentOver(LEFT_DRIVE_MOTOR_B_PDP_CHAN, power);
            if (tempPowerRatio < individualPowerRatio) { individualPowerRatio = tempPowerRatio; }
            power = individualPowerRatio;
            break;
        default: printf("WARNING: current not found to modify. In DriveTrain::ModifyCurrents()");
    }
    return power;
}

void DriveTrain::ConfigureShuffleboard()
{
    auto& functionalityTab = frc::Shuffleboard::GetTab("Functionality");
    auto& pidTab           = frc::Shuffleboard::GetTab("PID Values");
    auto& distancePIDLayout =
        pidTab.GetLayout("DriveStraight PID", "List Layout").GetLayout("Distance", "List Layout");
    auto& fieldLayout = pidTab.GetLayout("Field", "List Layout");

    _closedLoopErrorEntry = pidTab.Add("Left closed loop error", 0.0).GetEntry();

    _leftDriveEncoderEntry   = functionalityTab.Add("Left Drive Encoder", 0.0).GetEntry();
    _rightDriveEncoderEntry  = functionalityTab.Add("Right Drive Encoder", 0.0).GetEntry();
    _leftDriveDistanceEntry  = functionalityTab.Add("Left Drive Distance (ft)", 0.0).GetEntry();
    _rightDriveDistanceEntry = functionalityTab.Add("Drive Drive Distance (ft)", 0.0).GetEntry();
    _ratioDriveEntry         = functionalityTab.Add("Ratio Drive", _ratioDrive).GetEntry();

    _ratioDriveEntry = functionalityTab.Add("Ratio Drive", _ratioDrive).GetEntry();

    _odEntryX       = fieldLayout.Add("odometry x", 0.0).GetEntry();
    _odEntryY       = fieldLayout.Add("odometry y", 0.0).GetEntry();
    _odEntryT       = fieldLayout.Add("odometry angle", 0.0).GetEntry();
    _navXYawEntry   = fieldLayout.Add("navx yaw", 0.0).GetEntry();
    _navXPitchEntry = fieldLayout.Add("navx pitch", 0.0).GetEntry();
    _navXRollEntry  = fieldLayout.Add("navx roll", 0.0).GetEntry();

    _dsDistEntry = distancePIDLayout.Add("drivestraight distance", 4.0).GetEntry();
}

void DriveTrain::UpdateShuffleboard()
{
    _closedLoopErrorEntry.SetDouble(_leftPrimary.GetClosedLoopError());

    _odEntryX.SetDouble((double)(_odometry.GetPose().X()));
    _odEntryY.SetDouble((double)(_odometry.GetPose().Y()));
    _odEntryT.SetDouble((double)(_odometry.GetPose().Rotation().Degrees()));
    _navXYawEntry.SetDouble(_sensorControl.GetNavXYaw());
    _navXPitchEntry.SetDouble(_sensorControl.GetNavXPitch());
    _navXRollEntry.SetDouble(_sensorControl.GetNavXRoll());
}

const frc::DifferentialDriveKinematics& DriveTrain::GetDriveKinematics()
{
    return _driveKinematics;
}
