#include "commands/autonomous.h"

#include <frc/SmartDashboard/SmartDashboard.h>

using namespace AutoConstants;

frc2::CommandPtr autos::ShootOne(DriveSubsystem* drive, ShootySubsystem* shoot) {
  return frc2::cmd::Sequence(
    frc2::FunctionalCommand(
            //run once
            [shoot] {shoot->fire(false);},
            //run continuously
            [shoot] {shoot->SetMotorSpeed(1800, 1800);},
            //run on command end
            [shoot](bool interrupted) {shoot->fire(true);},
            //check for true
             [shoot] {
                return shoot->SetMotorSpeed(1800, 1800);
             },
            {shoot}).ToPtr(),
    frc2::cmd::Wait(0.5_s).AsProxy().AndThen(frc2::InstantCommand(
            [shoot] {
                shoot->fire(false);
                shoot->SetMotorSpeed(0, 0);
            },

            {shoot}).ToPtr()),
    frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive while the command is executing
             [drive] {drive->Drive(-0.1_mps, 0_mps, 0_rad_per_s, false, true);},
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
             //distance to drive
             [drive] {
              return drive->GetPose().X() <= -1.0_m;
               
             },
             // Requires the drive subsystem
             {drive}).ToPtr()
  );
}

frc2::CommandPtr autos::ShootTwo(DriveSubsystem* drive, ShootySubsystem* shoot, IntakeSubsystem* intake) {
  return frc2::cmd::Sequence(
    frc2::FunctionalCommand(
            //run once
            [shoot] {shoot->fire(false);},
            //run continuously
            [shoot] {shoot->SetMotorSpeed(1800, 1800);},
            //run on command end
            [shoot](bool interrupted) {shoot->fire(true);},
            //check for true
             [shoot] {
                return shoot->SetMotorSpeed(1800, 1800);
             },
            {shoot}).ToPtr(),
    frc2::cmd::Wait(0.5_s).AsProxy().AndThen(frc2::InstantCommand(
            //stop shooting
            [shoot] {
                shoot->fire(false);
                shoot->SetMotorSpeed(0, 0);
            },

            {shoot}).ToPtr()),
    frc2::InstantCommand(
        //run intake
        [intake] {intake->SetIntakeMotorSpeed(-0.6);},
                {intake}).ToPtr(),
    frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive backward while the command is executing
             [drive] {
                drive->Drive(-0.1_mps, 0_mps, 0_rad_per_s, false, true);
             },
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
             //distance to drive
             [drive] {
              return drive->GetPose().X() <= -2.0_m;
             },
             // Requires the drive subsystem
             {drive}).ToPtr(), 
    
    frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { 
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg});
             },
             // Drive forward while the command is executing 
             [drive] {drive->Drive(0.1_mps, 0_mps, 0_rad_per_s, false, true);},
             // stop driving 
             [drive](bool interrupted) {
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                },
             //distance to drive
             [drive] {
              return drive->GetPose().X() >= 2.0_m;
             },
             // Requires the drive subsystem
             {drive}).ToPtr(),
    frc2::InstantCommand(
        //stop intake
        [intake] {
                intake->SetIntakeMotorSpeed(0);
        },
                {intake}).ToPtr(),
    frc2::FunctionalCommand(
            //run once
            [shoot] {
                shoot->fire(false);
            },
            //run continuously
            [shoot, drive] {
                shoot->SetMotorSpeed(1800, 1800);
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);

            },
            //run on command end
            [shoot](bool interrupted) {shoot->fire(true);},
            //check for true
             [shoot] {
                return shoot->SetMotorSpeed(1800, 1800);
             },
            {shoot, drive}).ToPtr(),
    frc2::cmd::Wait(0.5_s).AsProxy().AndThen(frc2::InstantCommand(
            //stop shooting
            [shoot] {
                shoot->fire(false);
                shoot->SetMotorSpeed(0, 0);
            },

            {shoot}).ToPtr())
        );

}

frc2::CommandPtr autos::EpicShooterThreeYeahBaby(DriveSubsystem* drive, ShootySubsystem* shoot, IntakeSubsystem* intake) {
        return frc2::cmd::Sequence(
    frc2::FunctionalCommand(
                [shoot] {shoot->fire(false);},
            //run continuously
            [shoot] {shoot->SetMotorSpeed(1800, 1800);},
            //run on command end
            [shoot](bool interrupted) {shoot->fire(true);},
            //check for true
             [shoot] {
                return shoot->SetMotorSpeed(1800, 1800);
             },
            {shoot}).ToPtr(),
    frc2::cmd::Wait(0.5_s).AsProxy().AndThen(frc2::InstantCommand(
            //stop shooting
            [shoot] {
                shoot->fire(false);
                shoot->SetMotorSpeed(0, 0);
            },

            {shoot}).ToPtr()),
    frc2::InstantCommand(
        //run intake
        [intake] {intake->SetIntakeMotorSpeed(-0.6);},
                {intake}).ToPtr(),
    frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive backward while the command is executing
             [drive] {
                drive->Drive(-0.1_mps, 0_mps, 0_rad_per_s, false, true);
             },
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
             //distance to drive
             [drive] {
              return drive->GetPose().X() <= -2.0_m;
             },
             // Requires the drive subsystem
             {drive}).ToPtr(), 
    
    frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { 
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg});
             },
             // Drive forward while the command is executing 
             [drive] {drive->Drive(0.1_mps, 0_mps, 0_rad_per_s, false, true);},
             // stop driving 
             [drive](bool interrupted) {
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                },
             //distance to drive
             [drive] {
              return drive->GetPose().X() >= 2.0_m;
             },
             // Requires the drive subsystem
             {drive}).ToPtr(),
    frc2::InstantCommand(
        //stop intake
        [intake] {
                intake->SetIntakeMotorSpeed(0);
        },
                {intake}).ToPtr(),
    frc2::FunctionalCommand(
            //run once
            [shoot] {
                shoot->fire(false);
            },
            //run continuously
            [shoot, drive] {
                shoot->SetMotorSpeed(1800, 1800);
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);

            },
            //run on command end
            [shoot](bool interrupted) {shoot->fire(true);},
            //check for true
             [shoot] {
                return shoot->SetMotorSpeed(1800, 1800);
             },
            {shoot, drive}).ToPtr(),
    frc2::cmd::Wait(0.5_s).AsProxy().AndThen(frc2::InstantCommand(
            //stop shooting
            [shoot] {
                shoot->fire(false);
                shoot->SetMotorSpeed(0, 0);
            },

            {shoot}).ToPtr()),
    //third thingy
    frc2::InstantCommand(
        //run intake
        [intake] {intake->SetIntakeMotorSpeed(-0.6);},
                {intake}).ToPtr(),
    frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive sideways while the command is executing
             [drive] {
                if(drive->GetPose().Y() >= 1.45_m)
                {
                  drive->Drive(-0.2_mps, 0.3_mps, 0_rad_per_s, false, true);
                }
                else{
                  drive->Drive(-0.2_mps, 0.0_mps, 0_rad_per_s, false, true);
                }
             },
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
             //distance to drive
             [drive] {
              return drive->GetPose().X() <= -2.0_m;
              
             },
             // Requires the drive subsystem
             {drive}).ToPtr(), 
   

    frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { 
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg});
             },
             // Drive forward while the command is executing 
             [drive] {
                if(drive->GetPose().Y() >= -1.2_m)
                {
                  drive->Drive(0.2_mps, -0.3_mps, 0_rad_per_s, false, true);
                }
                else{
                  drive->Drive(0.2_mps, 0.0_mps, 0_rad_per_s, false, true);
                }
             },
             // stop driving 
             [drive](bool interrupted) {
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                },
             //distance to drive
             [drive] {
              return drive->GetPose().X() >= 2.0_m;
             },
             // Requires the drive subsystem
             {drive}).ToPtr(),
    frc2::InstantCommand(
        //stop intake
        [intake] {
                intake->SetIntakeMotorSpeed(0);
        },
                {intake}).ToPtr(),
    frc2::FunctionalCommand(
            //run once
            [shoot] {
                shoot->fire(false);
            },
            //run continuously
            [shoot, drive] {
                shoot->SetMotorSpeed(1800, 1800);
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);

            },
            //run on command end
            [shoot](bool interrupted) {shoot->fire(true);},
            //check for true
             [shoot] {
                return shoot->SetMotorSpeed(1800, 1800);
             },
            {shoot, drive}).ToPtr(),
    frc2::cmd::Wait(0.5_s).AsProxy().AndThen(frc2::InstantCommand(
            //stop shooting
            [shoot] {
                shoot->fire(false);
                shoot->SetMotorSpeed(0, 0);
            },

            {shoot}).ToPtr())
    );
}
    