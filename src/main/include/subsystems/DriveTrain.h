#pragma once

#include <ctre/phoenix/motorcontrol/TalonFXSensorCollection.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/PowerDistribution.h>
#include <frc/Solenoid.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>

class SensorBoard;

#include "utils/DRIVE_CONSTANTS.h"
#include "utils/PORTS.h"

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class DriveTrain : public frc2::SubsystemBase {
   public:
    DriveTrain(SensorBoard& sensorControl);
    enum Wheels { kLeftWheels, kRightWheels, kAllWheels };

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    /**
     * arcade drive
     * @param thrust
     * @param rotate
     */
    void ArcadeDrive(double thrust, double rotate);

    /**
     * adjusts motor values based off max power
     */
    void MaxSpeedAdjustment(double& leftvalue, double& rightvalue);

    /**
     * returns how much the thrust value should be adjusted
     */
    double GetDeadbandAdjustment(double value);

    /**
     * adjusts joystick sensitivity using a cubic for smoother driving
     */
    double GetCubicAdjustment(double value, double adjustmentConstant);

    /**
     * resets both encoders to read position of 0
     */
    void ResetEncoders();

    /**
     * calculates & returns average distance of the two encoders
     */
    double GetAverageEncoderDistance();

    /**
     * change in left encoder position
     */
    double GetLeftEncoderValue();

    /**
     * change in right encoder position
     */
    double GetRightEncoderValue();

    /**
     * left encoder value
     */
    double GetRawLeftEncoderValue();

    /**
     * right encoder value
     */
    double GetRawRightEncoderValue();

    void UpdateEncoderValues();

    void SetHighGear();
    void SetLowGear();
    bool IsHighGear();

    double GetDriveStraightAngleP();
    double GetDriveStraightAngleI();
    double GetDriveStraightAngleD();

    double GetDriveStraightDistanceP();
    double GetDriveStraightDistanceI();
    double GetDriveStraightDistanceD();

    double GetDriveStraightDistance();

    double GetPivotP();
    double GetPivotI();
    double GetPivotD();

    units::meter_t GetRightDistance();
    units::meter_t GetLeftDistance();

    void ResetDriveEncoders();

    frc::Pose2d GetPose();

    void ResetOdometry(frc::Pose2d pose);
    void TankDriveVolts(units::volt_t left, units::volt_t right);
    void ZeroDriveVolts();

    void SetDriveValues(DriveTrain::Wheels wheel, double value, bool isAutonomous);

    // brownout
    double GetVoltage();
    double GetTotalCurrent();
    void UpdateCurrent();
    double GetCurrent(int channel);
    double ModifyCurrent(int channel, double value);
    double CheckMotorCurrentOver(int channel, double power);

    frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();

    void SetLeftRightSpeeds(units::meters_per_second_t left_speed,
                            units::meters_per_second_t right_speed);

    const frc::DifferentialDriveKinematics& GetDriveKinematics();

   private:
    SensorBoard& _sensorControl;
    frc::PowerDistribution _pdp{PDP_ID, PDP_MODULE_TYPE};

    WPI_TalonFX _leftPrimary{LEFT_DRIVE_MOTOR_ID};
    WPI_TalonFX _rightPrimary{RIGHT_DRIVE_MOTOR_ID};
    WPI_TalonFX _leftSecondary{LEFT_DRIVE_MOTOR_2_ID};
    WPI_TalonFX _rightSecondary{RIGHT_DRIVE_MOTOR_2_ID};

    frc::DifferentialDriveKinematics const _driveKinematics{DRIVETRAIN_TRACK_WIDTH};

    frc::MotorControllerGroup _leftMotors{_leftPrimary, _leftSecondary};
    frc::MotorControllerGroup _rightMotors{_rightPrimary, _rightSecondary};

    frc::DifferentialDrive _drive{_leftMotors, _rightMotors};

    frc::DifferentialDriveOdometry _odometry;

    frc::Solenoid _gearSolenoid{
        PNEUMATICS_MODULE_ID, PNEUMATICS_MODULE_TYPE, GEAR_SHIFT_SOLENOID_PORT};

    // encoders
    TalonFXSensorCollection _leftDriveEncoder{_leftPrimary.GetSensorCollection()};
    TalonFXSensorCollection _rightDriveEncoder{_rightPrimary.GetSensorCollection()};

    double _lastRightEncoderValue    = 0.0;
    double _lastLeftEncoderValue     = 0.0;
    double _currRightEncoderValue    = 0.0;
    double _currLeftEncoderValue     = 0.0;
    double _initialRightEncoderValue = 0.0;
    double _initialLeftEncoderValue  = 0.0;

    double _thrustSensitivity = 0;
    double _rotateSensitivity = 0;

    double _leftDriveACurrent, _leftDriveBCurrent, _rightDriveACurrent, _rightDriveBCurrent;
    bool _isHighGear = true;
    bool _lastOver;
    double _ratioDrive = MIN_RATIO_DRIVE_CURRENT;

    int _kSlot       = 0;
    int _kPIDIdx     = 0;
    int _timeout     = 0;
    double _left_ff  = 0.0;
    double _right_ff = 0.0;

    // joystick values
    double _rightJoystickXLastValue = 0.0;
    double _rightJoystickXCurrValue = 0.0;
    double _leftJoystickYLastValue  = 0.0;
    double _leftJoystickYCurrValue  = 0.0;

    nt::NetworkTableEntry _pPEntry, _pIEntry, _pDEntry;
    nt::NetworkTableEntry _leftDriveEncoderEntry, _rightDriveEncoderEntry;
    nt::NetworkTableEntry _leftDriveDistanceEntry, _rightDriveDistanceEntry;
    nt::NetworkTableEntry _minVoltEntry, _maxCurrentEntry;
    nt::NetworkTableEntry _ratioDriveEntry, _dsDistEntry, _closedLoopErrorEntry;
    nt::NetworkTableEntry _odEntryX, _odEntryY, _odEntryT;
    nt::NetworkTableEntry _navXYawEntry, _navXPitchEntry, _navXRollEntry;

    frc::SimpleMotorFeedforward<units::meters> _velocity_feedforward{
        DriveConstants::ks, DriveConstants::kv, DriveConstants::ka};

    void ConfigureShuffleboard();
    void UpdateShuffleboard();
};
