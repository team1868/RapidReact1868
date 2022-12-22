#include "subsystems/Intake.h"

#include <iostream>

Intake::Intake()
{
    _intakeRollersMotor.SetNeutralMode(NeutralMode::Coast);

#ifdef JO
    _indexFunnelMotorA.SetInverted(true);
    _indexFunnelMotorB.SetInverted(true);
#else
    _intakeRollersMotor.SetInverted(true);
    _intakeSolenoid.Set(false);
#endif
    std::cout << "end of intake" << std::endl;
}

void Intake::SetIntakeRollersPower(double power)
{
    _intakeRollersMotor.Set(power);
#ifdef JO
    _indexFunnelMotorB.Set(power);
    _indexFunnelMotorA.Set(power);
#endif
}

double Intake::GetIntakeRollersPower() { return _intakeRollersMotor.Get(); }

void Intake::RaiseIntakeArm()
{
#ifdef JO
#else
    _intakeSolenoid.Set(false);
#endif
}

void Intake::LowerIntakeArm()
{
#ifdef JO
#else
    _intakeSolenoid.Set(true);
#endif
}