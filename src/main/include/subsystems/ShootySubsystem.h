#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkPIDController.h>
#include <rev/SparkRelativeEncoder.h>

class ShootySubsystem : public frc2::SubsystemBase {
public:
    ShootySubsystem();
    void SetMotorSpeed(double speed);
    

private:
    rev::CANSparkMax m_motor{1, rev::CANSparkMax::MotorType::kBrushless};  // Replace '1' with the CAN ID of the Spark MAX
};