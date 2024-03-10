// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "LimelightHelpers.h"


using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  shooting = false; //default to not shooting
  isRed = true; //default to red alliance
  if(frc::DriverStation::GetAlliance().value() == frc::DriverStation::Alliance::kBlue)
  {
    isRed = false;
  }


  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
        //set default values based on joysticks
        double y = m_driverController.GetLeftY()*0.3;
        double x = m_driverController.GetLeftX()*0.3;
        double theta = m_driverController.GetRightX()*0.3;

        m_drive.Drive(
            -units::meters_per_second_t{frc::ApplyDeadband(y, OIConstants::kDriveDeadband)},
            -units::meters_per_second_t{frc::ApplyDeadband(x, OIConstants::kDriveDeadband)},
            -units::radians_per_second_t{frc::ApplyDeadband(theta, OIConstants::kDriveDeadband)},
            true, true);
      },
      {&m_drive}));

    //using the shooty subsystem default command to use for generic smartdashboard stuff
    m_ShootySubsystem.SetDefaultCommand(frc2::RunCommand(
        [this] {
            m_ShootySubsystem.smartDashboardParams();
            frc::SmartDashboard::PutNumber("Apriltag ID",LimelightHelpers::getFiducialID(""));
            frc::SmartDashboard::PutNumber("Apriltag TX",LimelightHelpers::getTX(""));
        },
        {&m_ShootySubsystem}));

}

void RobotContainer::ConfigureButtonBindings() {
    //set wheels to the X configuration
    /*
    frc2::JoystickButton(&m_driverController,
                       frc::XboxController::Button::kRightBumper)
        .WhileTrue(new frc2::RunCommand([this] { m_drive.SetX(); }, {&m_drive}));
        */
    
    //spin intake
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kA)
       .WhileFalse(new frc2::RunCommand([this] { m_IntakeSubsystem.SetIntakeMotorSpeed(0);})).WhileTrue(new frc2::RunCommand([this] {m_IntakeSubsystem.SetIntakeMotorSpeed(-.6);}));
    //fire note into amp
    frc2::JoystickButton(&m_driverController,
                         frc::XboxController::Button::kLeftBumper)
        .OnFalse(new frc2::InstantCommand([this] 
        { 
            shooting = false;
            m_ShootySubsystem.SetMotorSpeed(0.0, 0.0);
            m_ShootySubsystem.fire(false);
        })).WhileTrue(new frc2::RunCommand([this] 
        {
            if(!shooting){ //set tag to the correct ID
                if(isRed){
                    currentTag = 5;
                    
                }
                else{
                    currentTag = 6;
                }
                LimelightHelpers::setPriorityTagID("", currentTag);
                shooting = true;
            }
            if(m_ShootySubsystem.SetMotorSpeed(ampTopShooterSpeed, ampBottomShooterSpeed)){

                m_ShootySubsystem.fire(true);
            }
            else
            {
                m_ShootySubsystem.fire(false);
            }
        }));
    //Fire note into speaker
    frc2::JoystickButton(&m_driverController,
                         frc::XboxController::Button::kRightBumper)
        .OnFalse(new frc2::InstantCommand([this]
        { 
            shooting = false;
            m_ShootySubsystem.SetMotorSpeed(0.0, 0.0);
            m_ShootySubsystem.fire(false);
        })).WhileTrue(new frc2::RunCommand([this] 
        {
            if(!shooting){ //set tag to the correct ID
                if(isRed){
                    currentTag = 4;
                    
                }
                else{
                    currentTag = 7;
                }
                LimelightHelpers::setPriorityTagID("", currentTag);
                shooting = true;
            }
            if(m_ShootySubsystem.SetMotorSpeed(speakerTopShooterSpeed, speakerBottomShooterSpeed)){

                m_ShootySubsystem.fire(true);
            }
            else
            {
                m_ShootySubsystem.fire(false);
            }
        }));
    //Retract climber piston
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kX)
        .OnTrue(new frc2::InstantCommand([this] { m_ClimberSubsystem.retractPiston();}));
    //Extend climber piston
    frc2::JoystickButton(&m_driverController,
                        frc::XboxController::Button::kY)
        .OnTrue(new frc2::InstantCommand([this] { m_ClimberSubsystem.extendPiston();}));

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d{0_m, 0_m, 0_deg},
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d{1_m, 1_m}, frc::Translation2d{2_m, -1_m}},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d{3_m, 0_m, 0_deg},
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t{-std::numbers::pi},
                                        units::radian_t{std::numbers::pi});

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc::PIDController{AutoConstants::kPXController, 0, 0},
      frc::PIDController{AutoConstants::kPYController, 0, 0}, thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() { m_drive.Drive(0_mps, 0_mps, 0_rad_per_s, false, false); },
          {}));
}
