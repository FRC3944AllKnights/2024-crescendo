#include "subsystems/ClimberSubsystem.h"
#include "Constants.h"

ClimberSubsystem::ClimberSubsystem() {
    // Additional initialization if needed
}

void ClimberSubsystem::extendPiston() {    
     m_doubleSolenoid.Set(frc::DoubleSolenoid::kForward);
}

void ClimberSubsystem::retractPiston() {    
     m_doubleSolenoid.Set(frc::DoubleSolenoid::kReverse);
}