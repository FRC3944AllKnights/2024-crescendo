#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/PneumaticHub.h>
#include <frc/DoubleSolenoid.h>
#include "constants.h"

using namespace ClimberConstants;

class ClimberSubsystem : public frc2::SubsystemBase {
public:
    ClimberSubsystem();
    void extendPiston();
    void retractPiston();

private:
    frc::PneumaticHub m_PH{pneumaticHubCANID};
    frc::DoubleSolenoid m_doubleSolenoid= m_PH.MakeDoubleSolenoid(climberExtendChannel,climberRetractChannel);
};