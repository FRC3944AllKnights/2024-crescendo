#include "subsystems/IntakeSubsystem.h"

#include "subsystems/MAXSwerveModule.h"

#include "Constants.h"



IntakeSubsystem::IntakeSubsystem() {
    // Additional initialization if needed
    

    m_IntakeMotor.RestoreFactoryDefaults();


    m_IntakePIDController.SetP(1.5e-5);
    m_IntakePIDController.SetI(3.3e-7);
    m_IntakePIDController.SetD(0);
    m_IntakePIDController.SetFF(0.000015);
    m_IntakePIDController.SetOutputRange(-1, 1);

}

void IntakeSubsystem::SetIntakeMotorSpeed(double speed) {
   
    m_IntakePIDController.SetReference(speed, rev::ControlType::kVelocity);
    

}