#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/Servo.h>
#include "Constants.h"

using namespace ShooterConstants;

class ShootySubsystem : public frc2::SubsystemBase {
public:
    ShootySubsystem();
    bool SetMotorSpeed(double topspeed, double botspeed);
    void fire(bool fire);
    void smartDashboardParams();
    void resetShooterI();

private:
    rev::CANSparkMax m_ShootyMotorTop{shooterTopCANID, rev::CANSparkMax::MotorType::kBrushless};  // Replace '1' with the CAN ID of the Spark MAX
    rev::CANSparkMax m_ShootyMotorBottom{shooterBottonCANID, rev::CANSparkMax::MotorType::kBrushless};

    rev::SparkRelativeEncoder m_ShootyEncoderTop =
        m_ShootyMotorTop.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkPIDController m_ShootyPIDControllerTop =
        m_ShootyMotorTop.GetPIDController();

    rev::SparkRelativeEncoder m_ShootyEncoderBottom =
        m_ShootyMotorBottom.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkPIDController m_ShootyPIDControllerBottom =
        m_ShootyMotorBottom.GetPIDController();

    //controls shooter feeder
    rev::CANSparkMax m_LaunchMotorLeft{LaunchMotorLeftCANID, rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_LaunchMotorRight{LaunchMotorRightCANID, rev::CANSparkMax::MotorType::kBrushless};
};