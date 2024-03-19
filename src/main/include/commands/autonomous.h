#pragma once

#include <frc2/command/CommandPtr.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShootySubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "LimelightHelpers.h"

#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SwerveControllerCommand.h>

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "Constants.h"

namespace autos {
    frc2::CommandPtr ShootOne(DriveSubsystem* drive, ShootySubsystem* shoot);
    frc2::CommandPtr ShootTwo(DriveSubsystem* drive, ShootySubsystem* shoot, IntakeSubsystem* intake);
    frc2::CommandPtr EpicShooterThreeYeahBaby(DriveSubsystem* drive, ShootySubsystem* shoot, IntakeSubsystem* intake);
}