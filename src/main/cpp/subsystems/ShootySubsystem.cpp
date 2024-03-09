#include "subsystems/ShootySubsystem.h"
#include "Constants.h"
#include <frc/SmartDashboard/SmartDashboard.h>


ShootySubsystem::ShootySubsystem() {
    // Additional initialization if needed
    m_ShootyMotorTop.RestoreFactoryDefaults();
    m_ShootyMotorBottom.RestoreFactoryDefaults();

    m_ShootyMotorTop.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    m_ShootyMotorBottom.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);

    m_ShootyMotorTop.SetInverted(true);
    m_ShootyMotorBottom.SetInverted(true);

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

    leftServo.Set(leftServoInitPosition);
    rightServo.Set(rightServoInitPosition);

    frc::SmartDashboard::PutNumber("Set Top Shooter RPM", top_shooter_speed_);
    frc::SmartDashboard::PutNumber("Set Bottom Shooter RPM", bottom_shooter_speed_);

    frc::SmartDashboard::PutNumber("Set Left Servo Position", left_servo_);
    frc::SmartDashboard::PutNumber("Set Right Servo Position", right_servo_);
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
    if(topspeed <= m_ShootyEncoderTop.GetVelocity() and botspeed <= m_ShootyEncoderBottom.GetVelocity()){
        return true;
    }
    else{
        return false;
    }
}

void ShootySubsystem::fire(bool fire) {
    if(fire) //&& abs(m_ShootyEncoderTop.GetVelocity()) > top_shooter_speed_ && abs(m_ShootyEncoderBottom.GetVelocity()) > bottom_shooter_speed_)
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

void ShootySubsystem::smartDashboardParams() {
    top_shooter_speed_ = frc::SmartDashboard::GetNumber("Set Top Shooter RPM", 500.0);
    bottom_shooter_speed_ = frc::SmartDashboard::GetNumber("Set Bottom Shooter RPM", 500.0);
    left_servo_ = frc::SmartDashboard::GetNumber("Set Left Servo Position", 0.5);
    right_servo_ = frc::SmartDashboard::GetNumber("Set Right Servo Position", 0.5);

    //frc::SmartDashboard::PutNumber("Set Left Servo Position", left_servo_);
    //frc::SmartDashboard::PutNumber("Set Right Servo Position", right_servo_);
}