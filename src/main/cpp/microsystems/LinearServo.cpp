#include "microsystems/LinearServo.h"

#include <cmath>
#include <iostream>

LinearServo::LinearServo(int CHANNEL, double length, double speed)
    : _servo{CHANNEL}, _length{length}, _speed{speed}
{
    _servo.SetBounds(2.0, 1.8, 1.5, 1.2, 1.0);
}

void LinearServo::SetPosition(double setpoint)
{
    double setPos = std::clamp(setpoint, 0.0, _length);
    _servo.SetSpeed((setPos / _length * 2) - 1);
}

double LinearServo::GetPosition() { return _curPos; }

bool LinearServo::IsFinished() { return fabs(_curPos - _setPos) < 0.5; }

void LinearServo::UpdateCurPos(double dt)
{
    double dPos = _speed * dt;
    if (_curPos - dPos > _setPos) {
        _curPos -= dPos;
    } else if (_curPos + dPos < _setPos) {
        _curPos += dPos;
    } else {
        _curPos = _setPos;
    }
}
