#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>

class IntakeSubsystem : public frc2::SubsystemBase {
public:
    IntakeSubsystem();
    void SetIntakeMotorSpeed(double speed);

    

private:
    rev::CANSparkMax m_IntakeMotor{20, rev::CANSparkMax::MotorType::kBrushless};  // Replace '20' with the CAN ID of the Spark MAX

    rev::SparkRelativeEncoder m_Encoder =
        m_IntakeMotor.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor);
    rev::SparkPIDController m_IntakePIDController =
        m_IntakeMotor.GetPIDController();
};