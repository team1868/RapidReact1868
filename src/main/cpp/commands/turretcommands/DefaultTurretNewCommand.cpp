#include "commands/turretcommands/DefaultTurretNewCommand.h"

#include <iostream>

#include "microsystems/SensorBoard.h"
#include "subsystems/Climber.h"
#include "subsystems/Turret.h"

DefaultTurretNewCommand::DefaultTurretNewCommand(Turret& turret,
                                                 Climber& climber,
                                                 SensorBoard& sensorControl)
    : _turret{turret}, _climber{climber}, _sensorControl{sensorControl}
{
    AddRequirements({&_turret});
}

void DefaultTurretNewCommand::Initialize()
{
    _numFramesGone = 0;
    _numTimesConst = 0;

    _turret.SpinTurretMotor(0.0);

    _pos               = true;
    _spinUp            = true;
    _scanTurretDesired = false;
    _oldTargetDesired  = false;

    nt::NetworkTableInstance::GetDefault().GetTable("limelight")->PutNumber("ledMode", 3);
}

double DefaultTurretNewCommand::CheckBounds(double desiredAngle, double currEncAngle)
{
    if (desiredAngle < ENCODER_MIN) {
        desiredAngle = desiredAngle + FULL_TURRET_RANGE * POT_TO_ENC;
        if (desiredAngle > ENCODER_MAX) { return ENCODER_MIN; }
    } else if (desiredAngle > ENCODER_MAX) {
        desiredAngle = desiredAngle - FULL_TURRET_RANGE * POT_TO_ENC;
        if (desiredAngle < ENCODER_MIN) { return ENCODER_MAX; }
    } else if (desiredAngle == ENCODER_MAX || desiredAngle == ENCODER_MIN) {
        return currEncAngle;
    }

    return desiredAngle;
}

void DefaultTurretNewCommand::Execute()
{
    double currEncAngle = _turret.GetTurretAngleEnc();

    // stop alignment when in auto
    if (_turret.GetAutoFreezeTurret()) {
        _turret.SpinTurretMotor(0.0);
        return;
    } else if (_climber.GetTeleHardStopLimitSwitch()) {
        return;
    }

    if ((!_turret.GetZeroTurret() && !_turret.GetClimbTurret()) || _sensorControl.IsAuto()) {
        double currLimelightAngle = _turret.GetTargetAngle();
        double distance           = _turret.GetTargetDistance();

        double desiredEncAngle = currEncAngle + currLimelightAngle * ANGLE_TO_ENC;

        if (distance < 0.01) {
            _numFramesGone++;
            desiredEncAngle = currEncAngle;
        } else {
            _numFramesGone     = 0;
            desiredEncAngle    = CheckBounds(desiredEncAngle, currEncAngle);
            _scanTurretDesired = false;
        }

        if (distance > 0.0 || _numFramesGone == 0) {
            if (_scanTurretDesired == true || _oldTargetDesired == true) {
                _turret.SpinTurretMotor(0.0);
            }
            _scanTurretDesired = false;
            _oldTargetDesired  = false;
            _numFramesGone     = 0;
            _turret.GoToSetpoint(currEncAngle, desiredEncAngle);
        } else if (_scanTurretDesired == false) {  // remembering old target
            _oldTargetDesired  = true;
            _scanTurretDesired = false;
            if (_pos && currEncAngle < ENCODER_MAX - ANGLE_TO_ENC * 5) {
                _turret.GoToScanSetpoint(currEncAngle, ENCODER_MAX - ANGLE_TO_ENC);
                _spinUp = true;
            } else if (!_pos && currEncAngle > ENCODER_MIN + ANGLE_TO_ENC * 5) {
                _turret.GoToScanSetpoint(currEncAngle, ENCODER_MIN + ANGLE_TO_ENC);
                _spinUp = false;
            } else {
                _turret.SpinTurretMotor(0.0);
                _scanTurretDesired = true;
                _oldTargetDesired  = false;
            }
        }

        // default scanning
        if (_scanTurretDesired == true && _numFramesGone > 10) {
            _oldTargetDesired = false;
            if (currEncAngle > ENCODER_MAX - ANGLE_TO_ENC * 5) {
                _spinUp = false;
                _turret.GoToScanSetpoint(currEncAngle, ENCODER_MIN);
            } else if (currEncAngle < ENCODER_MIN + ANGLE_TO_ENC * 5) {
                _spinUp = true;
                _turret.GoToScanSetpoint(currEncAngle, ENCODER_MAX);
            } else {
                _turret.GoToScanSetpoint(currEncAngle, _spinUp ? ENCODER_MAX : ENCODER_MIN);
            }
        }

        if (distance > 0.0) {
            _scanTurretDesired = false;
            _numTimesConst++;
            if (currLimelightAngle != 0) {
                if (!_pos) { _numTimesConst = 0; }
                _pos = currLimelightAngle > 0.0;
            }
        }

        // LEDS
        if (distance >= 0.01 && abs(currLimelightAngle) < 1.5) {
            _sensorControl.SetDesiredColor(9);
            _sensorControl.SetTurretLEDOverride(false);
        } else {
            _sensorControl.SetTurretLEDOverride(true);
        }
    }
}

void DefaultTurretNewCommand::End(bool interrupted) { _turret.SpinTurretMotor(0.0); }
