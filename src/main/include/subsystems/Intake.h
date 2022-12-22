#pragma once

#include <ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h>
#include <frc/Solenoid.h>
#include <frc2/command/SubsystemBase.h>

#include "utils/PORTS.h"

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class Intake : public frc2::SubsystemBase {
   public:
    Intake();

    void Deploy();
    void RetractArm();
    void LowerArm(double speed);
    void SetIntakeRollersPower(double speed);
    double GetIntakeRollersPower();
    void RaiseIntakeArm();
    void LowerIntakeArm();

   private:
    WPI_VictorSPX _intakeRollersMotor{INTAKE_ROLLERS_MOTOR_ID};

#ifdef JO
    frc::Solenoid _intakeSolenoid{
        PNEUMATICS_MODULE_ID, PNEUMATICS_MODULE_TYPE, INTAKE_SOLENOID_FORWARD_PORT};

    WPI_VictorSPX _indexFunnelMotorA{INDEX_FUNNEL_MOTOR_A_ID};
    WPI_VictorSPX _indexFunnelMotorB{INDEX_FUNNEL_MOTOR_B_ID};

#else

    frc::Solenoid _intakeSolenoid{
        PNEUMATICS_MODULE_ID, PNEUMATICS_MODULE_TYPE, INTAKE_SOLENOID_FORWARD_PORT};
#endif
};
