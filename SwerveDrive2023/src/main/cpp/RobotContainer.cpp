// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

using namespace DriveConstants;

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
      std::cout << "cout in robot container" << std::endl;

  // Configure the button bindings
  ConfigureButtonBindings();
  m_drive.ZeroHeading();

m_drive.SetDefaultCommand(frc2::RunCommand(
      [this] {
      bool noJoystick = false;
      double safeX = Deadzone(m_driverController.GetLeftX());
      double safeY =  Deadzone(m_driverController.GetLeftY());
      double safeRot = Deadzone(m_driverController.GetRightX());
      bool fieldOrientated;
      if (m_driverController.GetRawAxis(3)> 0.15){
        fieldOrientated = false;
      }
      if (m_driverController.GetRawAxis(3)< 0.15){
        fieldOrientated = true;
      }
      if((safeX == 0) && (safeY == 0) && (safeRot == 0)) {
        noJoystick = true;
      }
      m_drive.Drive(units::meters_per_second_t(
                    -safeY * AutoConstants::kMaxSpeed),
                    units::meters_per_second_t(
                    -safeX * AutoConstants::kMaxSpeed),
                    units::radians_per_second_t(
                    -safeRot * std::numbers::pi * 1.5),
                    fieldOrientated,
                    noJoystick);
      },
      {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings() {
    //frc2::JoystickButton(&m_driverController, 2).OnTrue(m_drive.ZeroHeading());      
}

float RobotContainer::Deadzone(float x){
  if ((x < 0.1) &&  (x > -0.1)){
    x=0;
  }
  else if(x >= 0.1){
    x = x - 0.1;
  }
  else if(x <= -0.1){
    x = x + 0.1;
  }
  return(x);
}

/*RobotContainer::~RobotContainer(){
  //delete shootCmd;
}*/

//frc2::Command* RobotContainer::GetAutonomousCommand() {
//   // Set up config for trajectory
//   frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
//                                AutoConstants::kMaxAcceleration);
//   // Add kinematics to ensure max speed is actually obeyed
//   config.SetKinematics(m_drive.kDriveKinematics);

//   // An example trajectory to follow.  All units in meters.
//   auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
//       // Start at the origin facing the +X direction
//       frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
//       // Pass through these two interior waypoints, making an 's' curve path
//       {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
//       // End 3 meters straight ahead of where we started, facing forward
//       frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
//       // Pass the config
//       config);

//   frc::ProfiledPIDController<units::radians> thetaController{
//       AutoConstants::kPThetaController, 0, 0,
//       AutoConstants::kThetaControllerConstraints};

//   thetaController.EnableContinuousInput(units::radian_t(-std::numbers::pi),
//                                         units::radian_t(std::numbers::pi));

//   frc2::SwerveControllerCommand<4> swerveControllerCommand(
//       exampleTrajectory, [this]() { return m_drive.GetPose(); },

//       m_drive.kDriveKinematics,

//       frc2::PIDController(AutoConstants::kPXController, 0, 0),
//       frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController,

//       [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

//       {&m_drive});

//   // Reset odometry to the starting pose of the trajectory.
//   m_drive.ResetOdometry(exampleTrajectory.InitialPose());

//   // no auto
//   return new frc2::SequentialCommandGroup(
//       std::move(swerveControllerCommand), std::move(swerveControllerCommand),
//       frc2::InstantCommand(
//           [this]() {
//             m_drive.Drive(units::meters_per_second_t(0),
//                           units::meters_per_second_t(0),
//                           units::radians_per_second_t(0), false, false);
//           },
//           {}));
  //return null;
//}