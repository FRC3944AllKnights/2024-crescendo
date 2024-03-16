#pragma once

#include <rev/ColorSensorV3.h>
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/colorMatch.h>

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem();
    void SetIntakeMotorSpeed(double speed);
    bool GamePieceDetected();

private:
    rev::CANSparkMax m_IntakeMotor{20, rev::CANSparkMax::MotorType::kBrushless};  // Replace '20' with the CAN ID of the Spark MAX

    rev::SparkRelativeEncoder m_Encoder =
        m_IntakeMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkPIDController m_IntakePIDController =
        m_IntakeMotor.GetPIDController();

    //rev::ColorSensorV3 m_colorSensor{frc::I2C::Port::kOnboard};

    //rev::ColorMatch m_colorMatcher;
    //frc::Color kGamePiece = frc::Color(0.361, 0.524, 0.113);
    //frc::Color kBackGround = frc::Color(0.143, 0.427, 0.429);
    
};