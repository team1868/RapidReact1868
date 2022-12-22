#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTableEntry.h>

#include "microsystems/LinearServo.h"
#include "utils/PORTS.h"

class Turret;
class SensorBoard;

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class Shooter : public frc2::SubsystemBase {
   public:
    Shooter(Turret& turret, SensorBoard& sensorControl);

    void Periodic() override;

    // flywheel control
    void ConfigFlywheelPID(double pFac, double iFac, double dFac);
    void ConfigFlywheelF();
    void SetFlywheelPercentRPM(double rpm);
    void SetFlywheelPercentOutput(double power);
    void SetFlywheelRPM(double rpm);
    double GetFlywheelRPM();
    bool IsFlywheelAtSpeed(double rpm);
    double CalculateFlywheelVelocityDesired();
    double GetTestingVelocityDesired();

    // hood control
    void RaiseHood();
    void ResetHood();
    void LowerHood();
    void SetHoodHeight(int pos);
    int CalculateHoodHeight();
    double HoodHeightEquation(double distance);
    bool HoodIsAtSetpoint();
    double GetDesiredRPM();
    double GetDesiredHood();

    // Shuffleboard configuration
    void ConfigureShuffleboard();
    void UpdateShuffleboard();
    nt::NetworkTableEntry _hoodConstantEntry, _velConstantEntry;
    nt::NetworkTableEntry _flywheelDesiredVelEntry, _errorVelocityEntry, _actualVelocityEntry;
    nt::NetworkTableEntry _rpmDistOffsetEntry, _offsetDistanceEntry;
    nt::NetworkTableEntry _rpmEntry, _hoodEntry;

   private:
    WPI_TalonFX _flywheelMotor1{FLYWHEEL_MOTOR_1_ID};
    WPI_TalonFX _flywheelMotor2{FLYWHEEL_MOTOR_2_ID};

    SensorBoard& _sensorControl;
    Turret& _turret;
    // flywheel resources
    bool _atTargetSpeed        = false;
    int _numTimeAtSpeed        = 0;
    double _velConstant        = 0.0;
    double _flywheelDesiredVel = 1200.0;
    double _rpmDistOffset      = 0.583;

    // hood resources
#if defined(PRACTICEBOT) || defined(COMPBOT)
    LinearServo _hood1{HOOD_1_PORT, 150, 32}, _hood2{HOOD_2_PORT, 150, 32};
#endif
    double _prevTime;
    double _hoodSetpoint  = 50.0;
    double _hoodConstant  = 0.0;
    double _hoodTolerance = 2.0;
};
