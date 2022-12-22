#pragma once

#include <ctre/phoenix/motorcontrol/TalonFXSensorCollection.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>

#include "utils/PORTS.h"

class SensorBoard;

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class Climber : public frc2::SubsystemBase {
   public:
    Climber(SensorBoard& sensorControl);

    void Periodic() override;
    void Reinit(bool disengage);

    double GetEncoderValue();
    double GetRawEncoderValue();
    void UpdateEncoderValues();

    void SetTelescopeArmOutput(double power);
    void SetTargetPosition(double target);
    double GetTargetPosition();

    void EngagePivotHook();
    void DisengagePivotHook();

    double GetPivotArmEncoderValue();
    double GetPivotArmRawEncoderValue();
    void SetPivotArmOutput(double power);
    void SetPivotArmTargetPosition(double target);
    double GetPivotArmTargetPosition();

    void ConfigTelescopePID(double pFac, double iFac, double dFac);
    void ConfigPivotPID(double pFac, double iFac, double dFac);

    bool GetResetClimbEncoders();

    bool GetLeftLimitSwitch();
    bool GetRightLimitSwitch();
    bool GetLRLimitSwitchAND();
    bool GetHardStopLimitSwitch();
    bool GetTeleHardStopLimitSwitch();

   private:
    SensorBoard& _sensorControl;

    frc::Solenoid _pivotHookSolenoid{
        PNEUMATICS_MODULE_ID, PNEUMATICS_MODULE_TYPE, HOOK_SOLENOID_PORT};
    WPI_TalonFX _telescopingArmMotor{TELESCOPE_ARM_MOTOR_ID};
    WPI_TalonFX _pivotArmMotor{PIVOT_ARM_MOTOR_ID};

    TalonFXSensorCollection& _teleArmMotorEncoder{_telescopingArmMotor.GetSensorCollection()};

#ifdef COMPBOT
    TalonFXSensorCollection& _pivotArmMotorEncoder{_pivotArmMotor.GetSensorCollection()};
#endif

    double _lastEncoderValue    = 0.0;
    double _curEncoderValue     = 0.0;
    double _initialEncoderValue = 0.0;
    int _timeout                = 30;  // in ms

    double _targetPosition           = 0.0;
    double _lastPivotArmEncoderValue = 0.0;
    double _curPivotArmEncoderValue  = 0.0;
    double _initialPivotEncoderValue = 0.0;
    double _pivotArmTargetPosition   = 0.0;
    bool _resetClimbEncoder          = false;
    bool _teleHardStopLS, _pivotHardStopLS, _leftHookLS, _rightHookLS;

    void ConfigureShuffleboard();
    void UpdateShuffleboard();
    void UpdateLimitSwitchValues();
    void ResetEncoders();

    nt::NetworkTableEntry _targetPositionEntry, _setTargetPositionEntry, _curPositionEntry;
    nt::NetworkTableEntry _pivotArmTargetPositionEntry, _setTargetPivotArmPositionEntry,
        _curPivotArmPositionEntry;
    nt::NetworkTableEntry _climberErrorEntry, _pivotArmErrorEntry;
    nt::NetworkTableEntry _swingDirEntry, _swingChangedEntry;

    nt::NetworkTableEntry _resetClimbEncoderEntry;
    nt::NetworkTableEntry _climbEncoderEntry;

    // Climber limit switches
    frc::DigitalInput _leftHookLimitSwitch{LEFT_CLIMBER_LIMIT_SWITCH_PORT};
    frc::DigitalInput _rightHookLimitSwitch{RIGHT_CLIMBER_LIMIT_SWITCH_PORT};
    frc::DigitalInput _pivotHardStopLimitSwitch{PIVOT_HARD_STOP_LIMIT_SWITCH_PORT};
    frc::DigitalInput _teleHardStopLimitSwitch{TELE_HARD_STOP_LIMIT_SWITCH_PORT};
    nt::NetworkTableEntry _leftLimitSwitchEntry, _rightLimitSwitchEntry;
    nt::NetworkTableEntry _pivotHardStopLimitSwitchEntry, _teleHardStopLimitSwitchEntry;
    nt::NetworkTableEntry _climberUpEntry;
};
