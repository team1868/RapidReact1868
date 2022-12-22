#include "subsystems/Shooter.h"

#include <iostream>

#include "microsystems/SensorBoard.h"
#include "subsystems/Turret.h"
#include "utils/CONSTANTS.h"
#include "utils/SHOOTER_CONSTANTS.h"

Shooter::Shooter(Turret& turret, SensorBoard& sensorControl)
    : _sensorControl{sensorControl}, _turret{turret}
{
    _flywheelMotor2.Follow(_flywheelMotor1);

    _flywheelMotor1.SetInverted(false);
    _flywheelMotor1.SetNeutralMode(NeutralMode::Coast);
    _flywheelMotor2.SetInverted(true);
    _flywheelMotor2.SetNeutralMode(NeutralMode::Coast);

    _flywheelMotor1.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    _flywheelMotor1.ConfigPeakOutputForward(1);
    _flywheelMotor1.ConfigPeakOutputReverse(0);

    ConfigFlywheelPID(0.19, 0.0, 0.0);
    ConfigFlywheelF();

    _prevTime = _sensorControl.GetCurTime();
}

void Shooter::Periodic()
{
    UpdateShuffleboard();

#if defined(PRACTICEBOT) || defined(COMPBOT)
    if (!_turret.GetClimbTurret() && _turret.GetTurretAligned()) {
        _hoodSetpoint = CalculateHoodHeight();
    }

    _hood1.SetPosition(_hoodSetpoint);
    _hood2.SetPosition(_hoodSetpoint);

    double curTime = _sensorControl.GetCurTime();
    double dt      = curTime - _prevTime;
    _prevTime      = curTime;
    _hood1.UpdateCurPos(dt);
    _hood2.UpdateCurPos(dt);
#endif
}

void Shooter::ConfigureShuffleboard()
{
    auto& superstructureTab = _sensorControl.GetSuperstructureTab();
    _hoodConstantEntry      = superstructureTab.Add("Hood Constant", 0.0).GetEntry();
    _velConstantEntry       = superstructureTab.Add("Vel Constant", 0.0).GetEntry();
    _flywheelDesiredVelEntry =
        superstructureTab.Add("Desired Velocity!", _flywheelDesiredVel).GetEntry();
    _actualVelocityEntry = superstructureTab.Add("Actual Velocity", 0.0).GetEntry();
    _errorVelocityEntry  = superstructureTab.Add("Velocity Error", 0.0).GetEntry();
    _offsetDistanceEntry =
        superstructureTab.Add("Offset Dist (fed into functions)", 0.0).GetEntry();
    _rpmDistOffsetEntry =
        _sensorControl.GetDriverTab().Add("dist offset", 0.583).WithPosition(2, 1).GetEntry();
    _rpmEntry  = superstructureTab.Add("DESIRED RPM", 1600).GetEntry();
    _hoodEntry = superstructureTab.Add("DESIRED HOOD", 60).GetEntry();
}

void Shooter::UpdateShuffleboard()
{
    _velConstant  = _velConstantEntry.GetDouble(0.0);
    _hoodConstant = _hoodConstantEntry.GetDouble(0.0);

    _flywheelDesiredVel = _flywheelDesiredVelEntry.GetDouble(1200.0);
    _rpmDistOffset      = _rpmDistOffsetEntry.GetDouble(0.583);

    // WITHOUT 2 ft because @pioneer, distances were from fender to center of hub
    // Distance has to match distance from fender to edge of hub
    _offsetDistanceEntry.SetDouble(_turret.GetTargetDistance() + _rpmDistOffset);

    double velRPM = GetFlywheelRPM();
    _actualVelocityEntry.SetDouble(velRPM);
    _errorVelocityEntry.SetDouble(_flywheelDesiredVel - velRPM);
}

void Shooter::SetFlywheelPercentOutput(double power)
{
    _flywheelMotor1.Set(TalonFXControlMode::PercentOutput, std::clamp<double>(power, -1.0, 1.0));
}

void Shooter::SetFlywheelPercentRPM(double rpm) { SetFlywheelPercentOutput(rpm / MAX_FALCON_RPM); }

void Shooter::SetFlywheelRPM(double rpm)
{
    double motorRPM = std::clamp<double>(rpm, 0.0, MAX_FALCON_RPM) / FALCON_TO_RPM;
    _flywheelMotor1.Set(TalonFXControlMode::Velocity, motorRPM);
}

double Shooter::GetFlywheelRPM()
{
    // 0 means primary closed loop, raw sensor units per 100 ms
    return _flywheelMotor1.GetSelectedSensorVelocity(0) * FALCON_TO_RPM;
}

void Shooter::ConfigFlywheelPID(double pFac, double iFac, double dFac)
{
    _flywheelMotor1.Config_kP(FLYWHEEL_PID_LOOP_ID, pFac);
    _flywheelMotor1.Config_kI(FLYWHEEL_PID_LOOP_ID, iFac);
    _flywheelMotor1.Config_kD(FLYWHEEL_PID_LOOP_ID, dFac);
}

void Shooter::ConfigFlywheelF()
{
    double fFac = 1023 / (MAX_FALCON_RPM / FALCON_TO_RPM);
    _flywheelMotor1.Config_kF(FLYWHEEL_PID_LOOP_ID, fFac);
}

bool Shooter::IsFlywheelAtSpeed(double rpm)
{
    if (GetFlywheelRPM() > rpm * 0.98) {
        _atTargetSpeed = ++_numTimeAtSpeed >= 5;
    } else {
        _atTargetSpeed  = false;
        _numTimeAtSpeed = 0;
    }
    return _atTargetSpeed;
}

double Shooter::CalculateFlywheelVelocityDesired()
{
    // all in inches
    // double shotDistance = 6.0 + 12.0 * sqrt(pow(_turret.GetTargetDistance(), 2.0) - 25.0);
    double distance = _turret.GetTargetDistance() + _rpmDistOffset;
    return distance > 5.0 ? distance * (-1.1161 * distance + 114.1964) + 996.9196 + _velConstant
                          : 1650;
}

void Shooter::RaiseHood() { SetHoodHeight(100); }

void Shooter::ResetHood() { SetHoodHeight(67); }

void Shooter::LowerHood() { SetHoodHeight(0); }

void Shooter::SetHoodHeight(int pos)
{
    _hoodSetpoint =
        !_turret.GetClimbTurret() && _turret.GetTurretAligned() ? CalculateHoodHeight() : pos;
#if defined(PRACTICEBOT) || defined(COMPBOT)
    _hood1.SetPosition(_hoodSetpoint);
    _hood2.SetPosition(_hoodSetpoint);
#endif
}

int Shooter::CalculateHoodHeight()
{
    // +2 ft because @pioneer, distances were from fender to center of hub
    double distance = _turret.GetTargetDistance() + _rpmDistOffset;
    return distance > 5.0 ? HoodHeightEquation(distance) : 45;
}

double Shooter::HoodHeightEquation(double distance)
{
    return distance * (-0.0357 * distance + 3.5143) + 40.1214 + _hoodConstant;
}

bool Shooter::HoodIsAtSetpoint()
{
#if defined(PRACTICEBOT) || defined(COMPBOT)
    // returns if hood height is within tolerance
    return fabs(_hood1.GetPosition() - _hoodSetpoint) < _hoodTolerance;
#endif
    return true;
}

double Shooter::GetTestingVelocityDesired() { return _flywheelDesiredVel; }

double Shooter::GetDesiredRPM() { return _rpmEntry.GetDouble(1600); }

double Shooter::GetDesiredHood() { return _hoodEntry.GetDouble(60); }
