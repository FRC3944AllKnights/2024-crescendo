#include "subsystems/ShootySubsystem.h"

ShootySubsystem::ShootySubsystem() {
    // Additional initialization if needed
}

void ShootySubsystem::SetMotorSpeed(double speed) {
    m_motor.Set(speed);
}