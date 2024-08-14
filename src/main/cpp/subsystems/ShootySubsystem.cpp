#include "subsystems/ShootySubsystem.h"
#include "Constants.h"
#include <frc/SmartDashboard/SmartDashboard.h>


ShootySubsystem::ShootySubsystem() {
    // Additional initialization if needed
    m_ShootyMotorTop.RestoreFactoryDefaults();
    m_ShootyMotorBottom.RestoreFactoryDefaults();

    m_ShootyMotorTop.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    m_ShootyMotorBottom.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);

    m_ShootyMotorTop.SetInverted(true);
    m_ShootyMotorBottom.SetInverted(true);

    m_ShootyPIDControllerTop.SetP(shooterP);
    m_ShootyPIDControllerTop.SetI(shooterI);
    m_ShootyPIDControllerTop.SetD(shooterD);
    m_ShootyPIDControllerTop.SetFF(0.000015);
    m_ShootyPIDControllerTop.SetOutputRange(-1, 1);

    m_ShootyPIDControllerBottom.SetP(shooterP);
    m_ShootyPIDControllerBottom.SetI(shooterI);
    m_ShootyPIDControllerBottom.SetD(shooterD);
    m_ShootyPIDControllerBottom.SetFF(0.000015);
    m_ShootyPIDControllerBottom.SetOutputRange(-1, 1);
}

bool ShootySubsystem::SetMotorSpeed(double topspeed, double botspeed) {
    if (topspeed > 0.0 and botspeed > 0.0)
    {
        m_ShootyPIDControllerTop.SetReference(topspeed, rev::ControlType::kVelocity);
        m_ShootyPIDControllerBottom.SetReference(botspeed, rev::ControlType::kVelocity);
    }
    else { //hopefully prevent weird stuff at low speed
        m_ShootyMotorTop.Set(0);
        m_ShootyMotorBottom.Set(0);
    }
    if(topspeed - 5 < m_ShootyEncoderTop.GetVelocity() and botspeed - 5 < m_ShootyEncoderBottom.GetVelocity()){
        return true;
    }
    else{
        return false;
    }
}

void ShootySubsystem::fire(bool fire) {
    if(fire) //&& abs(m_ShootyEncoderTop.GetVelocity()) > top_shooter_speed_ && abs(m_ShootyEncoderBottom.GetVelocity()) > bottom_shooter_speed_)
    {
        m_LaunchMotorLeft.Set(-LaunchMotorSpeed);
        m_LaunchMotorRight.Set(LaunchMotorSpeed);
    }
    else
    {
        m_LaunchMotorLeft.Set(0.0);
        m_LaunchMotorRight.Set(0.0);
    }
}

void ShootySubsystem::smartDashboardParams() {

    frc::SmartDashboard::PutNumber("Top Shooter RPM", m_ShootyEncoderTop.GetVelocity());
    frc::SmartDashboard::PutNumber("Bottom Shooter RPM", m_ShootyEncoderBottom.GetVelocity());

    //frc::SmartDashboard::PutNumber("Set Left Servo Position", left_servo_);
    //frc::SmartDashboard::PutNumber("Set Right Servo Position", right_servo_);
}

void ShootySubsystem::resetShooterI(){
    m_ShootyPIDControllerBottom.SetIAccum(0.0);
    m_ShootyPIDControllerTop.SetIAccum(0.0);
}