#include "microsystems/SensorBoard.h"

#include <frc/geometry/Rotation2d.h>
#include <networktables/NetworkTable.h>
#include <iostream>

SensorBoard::SensorBoard()
    : _setupTab(frc::Shuffleboard::GetTab("Setup")),
      _superstructureTab(frc::Shuffleboard::GetTab("Superstructure")),
      _driverTab(frc::Shuffleboard::GetTab("Drive Team Control"))
{
    _timer.Start();
    _curTime  = GetTime();
    _curPitch = GetNavXPitch();
}

frc::ShuffleboardTab& SensorBoard::GetDriverTab() { return _driverTab; }
frc::ShuffleboardTab& SensorBoard::GetSetupTab() { return _setupTab; }
frc::ShuffleboardTab& SensorBoard::GetSuperstructureTab() { return _superstructureTab; }

void SensorBoard::SetBlockerDesired(bool desired) { _blockerDesired = desired; }
bool SensorBoard::IsBlockerDesired() { return _blockerDesired; }

void SensorBoard::SetIntaking(bool val) { _isCurIntaking = val; }
bool SensorBoard::IsIntaking() { return _isCurIntaking; }

void SensorBoard::SetAutoClimbing(bool val) { _curAutoClimbing = val; }
bool SensorBoard::IsAutoClimbing() { return _curAutoClimbing; }

void SensorBoard::SetCurrentlyClimbing(bool isClimbing) { _isCurrentlyClimbing = isClimbing; }
bool SensorBoard::IsCurrentlyClimbing() { return _isCurrentlyClimbing; }

void SensorBoard::SetAtTurretClimbAngle(bool isAtAngle) { _isAtClimbAngle = isAtAngle; }
bool SensorBoard::IsAtTurretClimbAngle() { return _isAtClimbAngle; }

double SensorBoard::GetTime() { return (double)_timer.Get(); }
double SensorBoard::GetCurTime() { return _curTime; }

void SensorBoard::SetAuto() { _isAuto = true; }
void SensorBoard::SetTeleop() { _isAuto = false; }
bool SensorBoard::IsAuto() { return _isAuto; }

void SensorBoard::SetDesiredColor(int desiredColor) { _desiredColor = desiredColor; }
int SensorBoard::GetDesiredColor() { return _desiredColor; }

void SensorBoard::SetTurretLEDOverride(bool LEDoverride) { _LEDoverride = LEDoverride; }
bool SensorBoard::GetTurretLEDOverride() { return _LEDoverride; }

void SensorBoard::SetIndexLEDDesired(int indexLEDdesired) { _indexLEDdesired = indexLEDdesired; }
bool SensorBoard::GetIndexLEDDesired() { return _indexLEDdesired; }

void SensorBoard::SetAlliance(frc::DriverStation::Alliance alliance) { _alliance = alliance; }
frc::DriverStation::Alliance SensorBoard::GetAlliance() { return _alliance; }

// Updates sensors
void SensorBoard::Update()
{
    // Update time
    _curTime = GetTime();

    // Update pitch and swing values
    double prevPitch = _curPitch;
    _curPitch        = GetNavXPitch();
    _prevSwing       = _curSwing;
    _curSwing        = _curPitch - prevPitch > 0;
}

void SensorBoard::UpdateShuffleboard() {}

// NavX wrappers
double SensorBoard::GetNavXPitch() { return _navX.GetPitch(); }
void SensorBoard::ResetNavX() { return _navX.Reset(); }

// Remapped because of vertical mount rio/navx
double SensorBoard::GetNavXYaw() { return _navX.GetAngle(); }
double SensorBoard::GetNavXRoll() { return _navX.GetRoll(); }

// Likely wrong due to reorientation
frc::Rotation2d SensorBoard::GetRotation2d() { return _navX.GetRotation2d(); }

bool SensorBoard::SwingDirJustChanged() { return _curSwing != _prevSwing; }

bool SensorBoard::IsPosSwing()
{
    // true if swinging away from bars
    return _curSwing;
}
