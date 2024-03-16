#include "subsystems/IntakeSubsystem.h"
#include "subsystems/MAXSwerveModule.h"
#include "Constants.h"



IntakeSubsystem::IntakeSubsystem() {
    //m_colorMatcher.AddColorMatch(kGamePiece);
    //m_colorMatcher.AddColorMatch(kBackGround);

    m_IntakeMotor.RestoreFactoryDefaults();

    m_IntakePIDController.SetP(1.5e-5);
    m_IntakePIDController.SetI(3.3e-7);
    m_IntakePIDController.SetD(0);
    m_IntakePIDController.SetFF(0.000015);
    m_IntakePIDController.SetOutputRange(-1, 1);
}

void IntakeSubsystem::SetIntakeMotorSpeed(double speed) {
    if (GamePieceDetected()){
       //m_IntakePIDController.SetReference(speed, rev::ControlType::kVelocity);
       m_IntakeMotor.Set(speed);
    }
    else{
        //m_IntakePIDController.SetReference(0, rev::ControlType::kVelocity);
        m_IntakeMotor.Set(0);
    }
}

bool IntakeSubsystem::GamePieceDetected(){
    double confidence = 0.1;
    //frc::Color detectedColor = m_colorSensor.GetColor();
    //frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);

    //if (matchedColor == kGamePiece){
    //    return true;
    //}
    return true; //bypass logic for now since color sensor isn't set up
}

