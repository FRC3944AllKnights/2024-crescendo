#include "subsystems/ShootySubsystem.h"
#include "Constants.h"



ShootySubsystem::ShootySubsystem() {
    // Additional initialization if needed
    m_ShootyMotorTop.RestoreFactoryDefaults();
    m_ShootyMotorBottom.RestoreFactoryDefaults();

    m_ShootyPIDControllerTop.SetP(1.5e-5);
    m_ShootyPIDControllerTop.SetI(3.3e-7);
    m_ShootyPIDControllerTop.SetD(0);
    m_ShootyPIDControllerTop.SetFF(0.000015);
    m_ShootyPIDControllerTop.SetOutputRange(-1, 1);

    m_ShootyPIDControllerBottom.SetP(1.5e-5);
    m_ShootyPIDControllerBottom.SetI(3.3e-7);
    m_ShootyPIDControllerBottom.SetD(0);
    m_ShootyPIDControllerBottom.SetFF(0.000015);
    m_ShootyPIDControllerBottom.SetOutputRange(-1, 1);
}

void ShootySubsystem::SetMotorSpeed(double speed) {
    if (speed > 0.0)
    {
        m_ShootyPIDControllerTop.SetReference(speed, rev::ControlType::kVelocity);
        m_ShootyPIDControllerBottom.SetReference(speed, rev::ControlType::kVelocity);
    }
    else { //hopefully prevent weird stuff at low speed
        m_ShootyMotorTop.Set(0);
        m_ShootyMotorBottom.Set(0);
    }
}

void ShootySubsystem::fire(bool fire) {
    if(fire)
    {
        leftServo.Set(leftServoFirePosition);
        rightServo.Set(rightServoFirePosition);
    }
    else
    {
        leftServo.Set(leftServoInitPosition);
        rightServo.Set(rightServoInitPosition);
    }
}