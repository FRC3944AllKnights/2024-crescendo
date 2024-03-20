#include "subsystems/IntakeSubsystem.h"
#include "subsystems/MAXSwerveModule.h"
#include "Constants.h"
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/util/color.h>


IntakeSubsystem::IntakeSubsystem() {
    m_colorMatcher.AddColorMatch(kGamePiece);
    m_colorMatcher.AddColorMatch(kBackGround);

    m_IntakeMotor.RestoreFactoryDefaults();
    
    m_IntakePIDController.SetP(1.5e-5);
    m_IntakePIDController.SetI(3.3e-7);
    m_IntakePIDController.SetD(0);
    m_IntakePIDController.SetFF(0.000015);
    m_IntakePIDController.SetOutputRange(-1, 1);
    
    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();

    SetColorLED(0, 0, 255);


}

void IntakeSubsystem::SetIntakeMotorSpeed(double speed) {
    if (!GamePieceDetected()){
       //m_IntakePIDController.SetReference(speed, rev::ControlType::kVelocity);
       m_IntakeMotor.Set(speed);
       SetColorLED(255, 227, 18);

    }
    else{
        //m_IntakePIDController.SetReference(0, rev::ControlType::kVelocity);
        m_IntakeMotor.Set(0);
        SetColorLED(0, 0, 255);
    }
}

bool IntakeSubsystem::GamePieceDetected(){
    double confidence = 0.1;
    frc::Color detectedColor = m_colorSensor.GetColor();
    frc::SmartDashboard::PutNumber("Color R", detectedColor.red);
    frc::SmartDashboard::PutNumber("Color G", detectedColor.green);
    frc::SmartDashboard::PutNumber("Color B", detectedColor.blue);

    frc::Color matchedColor = m_colorMatcher.MatchClosestColor(detectedColor, confidence);
    frc::SmartDashboard::PutNumber("MATCHED R", matchedColor.red);
    frc::SmartDashboard::PutNumber("MATCHED G", matchedColor.green);
    frc::SmartDashboard::PutNumber("MATCHED B", matchedColor.blue);

    if (matchedColor == kGamePiece){
        return true;
    }
    else {
    return false;
    }
    //return true; //bypass logic for now since color sensor isn't set up
}

void IntakeSubsystem::SetColorLED(int R, int G, int B){
    for (int i = 0; i < kLength; i++) {
   m_ledBuffer[i].SetRGB(R, G, B);
    }

    m_led.SetData(m_ledBuffer);


}