#pragma once

#include <frc/Servo.h>

class LinearServo {
   public:
    // length in mm and speed in mm/s
    LinearServo(int CHANNEL, double length, double speed);

    void UpdateCurPos(double dt);
    void SetPosition(double setpoint);
    double GetPosition();
    bool IsFinished();

   private:
    frc::Servo _servo;
    double _length;
    double _speed;
    double _setPos;
    double _curPos;
};
