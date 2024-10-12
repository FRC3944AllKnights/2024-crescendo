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
    frc2::cmd::Wait(1.0_s).AsProxy().AndThen(frc2::InstantCommand(
            [shoot] {
                shoot->fire(false);
                shoot->SetMotorSpeed(0, 0);
            },

            {shoot}).ToPtr()),
    frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive while the command is executing
             [drive] {drive->Drive(-0.2_mps, 0_mps, 0_rad_per_s, false, true);},
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


frc2::CommandPtr autos::TurnLeftShootOne(DriveSubsystem* drive, ShootySubsystem* shoot) {
return frc2::cmd::Sequence(

     frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive while the command is executing
             [drive] {drive->Drive(0.0_mps, 0_mps, 0.3_rad_per_s, false, true);},
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
             //distance to drive
             [drive] {
              return drive->GetHeading() <= 225_deg;
               
             },
             // Requires the drive subsystem
             {drive}).ToPtr(),

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
    frc2::cmd::Wait(1.0_s).AsProxy().AndThen(frc2::InstantCommand(
            [shoot] {
                shoot->fire(false);
                shoot->SetMotorSpeed(0, 0);
            },

            {shoot}).ToPtr()),
    frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, 0_m, 0_deg}); },
             // Drive while the command is executing
             [drive] {drive->Drive(-0.2_mps, 0_mps, 0_rad_per_s, false, true);},
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
    frc2::cmd::Wait(1.0_s).AsProxy().AndThen(frc2::InstantCommand(
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
              return drive->GetPose().X() <= -1.8_m;
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
             [drive] {drive->Drive(0.2_mps, 0_mps, 0_rad_per_s, false, true);},
             // stop driving 
             [drive](bool interrupted) {
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                },
             //distance to drive
             [drive] {
              return drive->GetPose().X() >= 1.8_m;
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
    frc2::cmd::Wait(1.0_s).AsProxy().AndThen(frc2::InstantCommand(
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
    //REV AND FIRE
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
    //TURN ON INTAKE
    frc2::cmd::Wait(0.1_s).AsProxy().AndThen(frc2::InstantCommand(
        //run intake
        [intake] {intake->SetIntakeMotorSpeed(-0.6);},
                {intake}).ToPtr()),
    //DRIVE BACKWARD TO SECOND NOTE
    frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, drive->GetPose().Y(), drive->GetPose().Rotation()}); },
             // Drive backward while the command is executing
             [drive] {
                drive->Drive(-xSpeed, 0_mps, 0_rad_per_s, false, true);
             },
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
             //distance to drive
             [drive] {
              return drive->GetPose().X() <= -1.7_m;
             },
             // Requires the drive subsystem
             {drive}).ToPtr(), 
    //DRIVE FORWARD TO SHOOT SECOND NOTE
    frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { 
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                drive->ResetOdometry(frc::Pose2d{0_m, drive->GetPose().Y(), drive->GetPose().Rotation()});
             },
             // Drive forward while the command is executing 
             [drive] {drive->Drive(xSpeed, 0_mps, 0_rad_per_s, false, true);},
             // stop driving 
             [drive](bool interrupted) {
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                },
             //distance to drivee
             [drive] {
              return drive->GetPose().X() >= 1.5_m;
             },
             // Requires the drive subsystem
             {drive}).ToPtr(),
    //TURN INTAKE OFF
    frc2::InstantCommand(
        //stop intake
        [intake] {
                intake->SetIntakeMotorSpeed(0);
        },
                {intake}).ToPtr(),
    //SHOOT SECOND NOTE
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
    //TURN ON INTAKE
    frc2::cmd::Wait(0.1_s).AsProxy().AndThen(frc2::InstantCommand(
        //run intake
        [intake] {intake->SetIntakeMotorSpeed(-0.6);},
                {intake}).ToPtr()),
    //DRIVE BACKWARD TO 3RD NOTE
    frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { drive->ResetOdometry(frc::Pose2d{0_m, drive->GetPose().Y(), drive->GetPose().Rotation()}); },
             // Drive sideways while the command is executing
             [drive] {
                if(drive->GetPose().Y() <= 1.35_m)
                {
                  drive->Drive(-xSpeed, ySpeed, 0_rad_per_s, false, true);
                }
                else{
                  drive->Drive(-xSpeed, 0_mps, 0_rad_per_s, false, true);
                }
             },
             // stop driving
             [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
             //distance to drive
             [drive] {
              return drive->GetPose().X() <= -1.8_m;
              
             },
             // Requires the drive subsystem
             {drive}).ToPtr(), 
   
    //DRIVE FORWARD TO SHOOT 3RD NOTE
    frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { 
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                drive->ResetOdometry(frc::Pose2d{0_m, drive->GetPose().Y(), drive->GetPose().Rotation()});
             },
             // Drive forward while the command is executing 
             [drive] {
                if(drive->GetPose().Y() >= 0.1_m)
                {
                  drive->Drive(xSpeed, -ySpeed, 0_rad_per_s, false, true);
                }
                else{
                  drive->Drive(xSpeed, 0_mps, 0_rad_per_s, false, true);
                }
             },
             // stop driving 
             [drive](bool interrupted) {
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                },
             //distance to drive
             [drive] {
              return drive->GetPose().X() >= 1.9_m;
             },
             // Requires the drive subsystem
             {drive}).ToPtr(),
    //STOP INTAKE
    frc2::InstantCommand(
        //stop intake
        [intake] {
                intake->SetIntakeMotorSpeed(0);
        },
                {intake}).ToPtr(),
    //SHOOT 3RD NOTE
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
    //TURN ON INTAKE
    frc2::cmd::Wait(0.1_s).AsProxy().AndThen(frc2::InstantCommand(
        //run intake
        [intake] {intake->SetIntakeMotorSpeed(-0.6);},
                {intake}).ToPtr()),

    //DRIVE BACKWARD TO 4TH NOTE
    frc2::FunctionalCommand(
        // Reset odometry on command start
        [drive] { drive->ResetOdometry(frc::Pose2d{0_m, drive->GetPose().Y(), drive->GetPose().Rotation()}); },
        // Drive sideways while the command is executing
        [drive] {
            if(drive->GetPose().Y() >= -1.3_m)
           {
             drive->Drive(-xSpeed, -ySpeed, 0_rad_per_s, false, true);
            }
            else{
              drive->Drive(-xSpeed, 0_mps, 0_rad_per_s, false, true);
            }
        },
        // stop driving
        [drive](bool interrupted) {drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);},
        //distance to drive
        [drive] {
          return drive->GetPose().X() <= -1.8_m;     
        },
        // Requires the drive subsystem
        {drive}).ToPtr(), 
   
    //DRIVE FORWARD TO SHOOT 4TH NOTE
    frc2::FunctionalCommand(
            // Reset odometry on command start
             [drive] { 
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                drive->ResetOdometry(frc::Pose2d{0_m, drive->GetPose().Y(), drive->GetPose().Rotation()});
             },
             // Drive forward while the command is executing 
             [drive] {
                if(drive->GetPose().Y() <= 0.0_m)
                {
                  drive->Drive(xSpeed, ySpeed, 0_rad_per_s, false, true);
                }
                else{
                  drive->Drive(xSpeed, 0_mps, 0_rad_per_s, false, true);
                }
             },
             // stop driving 
             [drive](bool interrupted) {
                drive->Drive(0_mps, 0_mps, 0_rad_per_s, false, true);
                },
             //distance to drive
             [drive] {
              return drive->GetPose().X() >= 1.9_m;
             },
             // Requires the drive subsystem
             {drive}).ToPtr(),
    //STOP INTAKE
    frc2::InstantCommand(
        [intake] {
                intake->SetIntakeMotorSpeed(0);
        },
                {intake}).ToPtr(),
    //SHOOT 4TH NOTE
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
    //STOP SHOOTING
    frc2::cmd::Wait(0.25_s).AsProxy().AndThen(frc2::InstantCommand(
            //stop shooting
            [shoot] {
                shoot->fire(false);
                shoot->SetMotorSpeed(0, 0);
            },

            {shoot}).ToPtr())
    );
}
    