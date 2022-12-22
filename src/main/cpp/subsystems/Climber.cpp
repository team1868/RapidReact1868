#include "subsystems/Climber.h"

#include <iostream>

#include "microsystems/SensorBoard.h"
#include "utils/CLIMBER_CONSTANTS.h"
#include "utils/HEIGHTS.h"

Climber::Climber(SensorBoard& sensorControl) : _sensorControl{sensorControl}
{
    std::cout << "start of climb constructor" << std::endl;

    _telescopingArmMotor.SetNeutralMode(NeutralMode::Brake);
    _telescopingArmMotor.ConfigClosedLoopPeakOutput(0, 1.0, _timeout);

#ifdef COMPBOT
    _pivotArmMotor.ConfigClosedLoopPeakOutput(0, 0.8, _timeout);
    _pivotArmMotor.SetNeutralMode(NeutralMode::Brake);
    _pivotArmMotor.SetInverted(false);
    _telescopingArmMotor.SetInverted(false);
#endif
    ConfigTelescopePID(0.15, 0.0, 14.0);
    ConfigPivotPID(0.5, 0.0, 0.0);
    ResetEncoders();
    ConfigureShuffleboard();
    std::cout << "end of climb constructor" << std::endl;
}

void Climber::Periodic()
{
    if (_resetClimbEncoder) { ResetEncoders(); }

    UpdateEncoderValues();
    UpdateLimitSwitchValues();

    UpdateShuffleboard();
}

void Climber::Reinit(bool disengage)
{
    disengage ? DisengagePivotHook() : EngagePivotHook();
    ResetEncoders();
}

void Climber::SetTelescopeArmOutput(double power)
{
    _telescopingArmMotor.Set(ControlMode::PercentOutput, power);
}

void Climber::SetPivotArmOutput(double power)
{
#ifdef COMPBOT
    _pivotArmMotor.Set(ControlMode::PercentOutput, power);
#endif
}

void Climber::ResetEncoders()
{
    // read current encoder values, store as initial encoder values
    _initialEncoderValue = _telescopingArmMotor.GetSelectedSensorPosition();

#ifdef COMPBOT
    _initialPivotEncoderValue = _pivotArmMotor.GetSelectedSensorPosition();
#endif
}

double Climber::GetEncoderValue()
{
    return _telescopingArmMotor.GetSelectedSensorPosition() - _initialEncoderValue;
}

double Climber::GetRawEncoderValue() { return _telescopingArmMotor.GetSelectedSensorPosition(); }

double Climber::GetPivotArmEncoderValue()
{
#ifdef COMPBOT
    return _pivotArmMotor.GetSelectedSensorPosition() - _initialPivotEncoderValue;
#endif
    return 0.0;
}

double Climber::GetPivotArmRawEncoderValue()
{
#ifdef COMPBOT
    return _pivotArmMotor.GetSelectedSensorPosition();
#endif
    return 0.0;
}

void Climber::UpdateEncoderValues()
{
    _lastEncoderValue = _curEncoderValue;
    _curEncoderValue  = GetEncoderValue();

    _lastPivotArmEncoderValue = _curPivotArmEncoderValue;
    _curPivotArmEncoderValue  = GetPivotArmEncoderValue();
}

double Climber::GetTargetPosition() { return _targetPosition; }

double Climber::GetPivotArmTargetPosition() { return _pivotArmTargetPosition; }

void Climber::SetTargetPosition(double target)
{
    _targetPosition = target > TELESCOPE_UPPER_LIMIT ? TELESCOPE_UPPER_LIMIT : target;
    _telescopingArmMotor.Set(ControlMode::Position, _targetPosition + _initialEncoderValue);
}

void Climber::SetPivotArmTargetPosition(double target)
{
#ifdef COMPBOT
    // clamp the values so we don't under or overextend
    _pivotArmTargetPosition = target > PIVOT_ARM_UPPER_LIMIT ? PIVOT_ARM_UPPER_LIMIT : target;
    _pivotArmMotor.Set(ControlMode::Position, _pivotArmTargetPosition + _initialPivotEncoderValue);
#endif
}

void Climber::ConfigTelescopePID(double pFac, double iFac, double dFac)
{
    _telescopingArmMotor.Config_kP(TELESCOPE_PID_LOOP_ID, pFac);
    _telescopingArmMotor.Config_kI(TELESCOPE_PID_LOOP_ID, iFac);
    _telescopingArmMotor.Config_kD(TELESCOPE_PID_LOOP_ID, dFac);
}

void Climber::ConfigPivotPID(double pFac, double iFac, double dFac)
{
#ifdef COMPBOT
    _pivotArmMotor.Config_kP(PIVOT_ARM_PID_LOOP_ID, pFac);
    _pivotArmMotor.Config_kI(PIVOT_ARM_PID_LOOP_ID, iFac);
    _pivotArmMotor.Config_kD(PIVOT_ARM_PID_LOOP_ID, dFac);
#endif
}

void Climber::EngagePivotHook() { _pivotHookSolenoid.Set(false); }

void Climber::DisengagePivotHook() { _pivotHookSolenoid.Set(true); }

void Climber::ConfigureShuffleboard()
{
    auto& climberTestTab = frc::Shuffleboard::GetTab("Climber Test");
    auto& teleLayout     = climberTestTab.GetLayout("Tele Arm", "List Layout");
    auto& pivotLayout    = climberTestTab.GetLayout("Pivot Arm", "List Layout");

    _swingDirEntry     = climberTestTab.Add("Swing dir", true).GetEntry();
    _swingChangedEntry = climberTestTab.Add("swing CHANGED", false).GetEntry();

    _targetPositionEntry = teleLayout.Add("Target tele Position", _targetPosition).GetEntry();
    _curPositionEntry    = teleLayout.Add("Get tele position", 0.0).GetEntry();
    _climberErrorEntry   = teleLayout.Add("Tele climber error", 0.0).GetEntry();

    _pivotArmTargetPositionEntry =
        pivotLayout.Add("Target pivot arm Position", _targetPosition).GetEntry();
    _curPivotArmPositionEntry = pivotLayout.Add("Get pivot arm position", 0.0).GetEntry();
    _pivotArmErrorEntry       = pivotLayout.Add("Pivot climber error", 0.0).GetEntry();

    auto& superstructureTab = _sensorControl.GetSuperstructureTab();
    _resetClimbEncoderEntry = superstructureTab.Add("Reset Climb Encoders", false)
                                  .WithWidget(frc::BuiltInWidgets::kToggleSwitch)
                                  .GetEntry();
    _climbEncoderEntry = superstructureTab.Add("Climb Encoder value", _curEncoderValue).GetEntry();

    auto& setupTab = _sensorControl.GetSetupTab();
    _pivotHardStopLimitSwitchEntry =
        setupTab.Add("Pivot Hard Stop Switches Working (should be false when triggered)", true)
            .GetEntry();
    _teleHardStopLimitSwitchEntry =
        setupTab.Add("Tele Hard Stop (should be false when triggered)", true).GetEntry();
    _leftLimitSwitchEntry  = setupTab.Add("Left Limit Switch", false).GetEntry();
    _rightLimitSwitchEntry = setupTab.Add("Right Limit Switch", false).GetEntry();

    _climberUpEntry = _sensorControl.GetDriverTab()
                          .Add("CLIMBER RESET", false)
                          .WithSize(2, 1)
                          .WithPosition(3, 1)
                          .GetEntry();
}

void Climber::UpdateShuffleboard()
{
    _swingDirEntry.SetBoolean(_sensorControl.IsPosSwing());
    _swingChangedEntry.SetBoolean(_sensorControl.SwingDirJustChanged());
    _climbEncoderEntry.SetDouble(_curEncoderValue);

    // show target and current positions
    _targetPositionEntry.SetDouble(_targetPosition);
    _curPositionEntry.SetDouble(_curEncoderValue);
    _pivotArmTargetPositionEntry.SetDouble(_pivotArmTargetPosition);
    _curPivotArmPositionEntry.SetDouble(_curPivotArmEncoderValue);

    // telescoping arm error
    _climberErrorEntry.SetDouble(_targetPosition - _curEncoderValue);
    _pivotArmErrorEntry.SetDouble(_pivotArmTargetPosition - _curPivotArmEncoderValue);

    _leftLimitSwitchEntry.SetBoolean(_leftHookLS);
    _rightLimitSwitchEntry.SetBoolean(_rightHookLS);
    _pivotHardStopLimitSwitchEntry.SetBoolean(_pivotHardStopLS);
    _teleHardStopLimitSwitchEntry.SetBoolean(_teleHardStopLS);
    _climberUpEntry.SetBoolean(!_pivotHardStopLS && !_teleHardStopLS);

    _resetClimbEncoder = _resetClimbEncoderEntry.GetBoolean(false);
}

void Climber::UpdateLimitSwitchValues()
{
    _pivotHardStopLS = _pivotHardStopLimitSwitch.Get();
    _teleHardStopLS  = _teleHardStopLimitSwitch.Get();
    _leftHookLS      = _leftHookLimitSwitch.Get();
    _rightHookLS     = _rightHookLimitSwitch.Get();
}

bool Climber::GetResetClimbEncoders() { return _resetClimbEncoder; }

bool Climber::GetHardStopLimitSwitch() { return _pivotHardStopLS; }
bool Climber::GetTeleHardStopLimitSwitch() { return _teleHardStopLS; }
bool Climber::GetLeftLimitSwitch() { return _leftHookLS; }
bool Climber::GetRightLimitSwitch() { return _rightHookLS; }
bool Climber::GetLRLimitSwitchAND() { return _leftHookLS || _rightHookLS; }
