// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LimelightPose.h"

LimelightPose::LimelightPose() = default;

// This method will be called once per scheduler run
void LimelightPose::Periodic() {

    //TODO make read actual values
    botPose = frc::SmartDashboard::GetNumberArray("botpose", std::span<const double>({0, 0, 0, 0, 0, 0}));
    
    frc::SmartDashboard::PutNumber("BOTPOSE1", botPose[0]);
    frc::SmartDashboard::PutNumber("BOTPOSE2", botPose[1]);
    frc::SmartDashboard::PutNumber("BOTPOSE3", botPose[2]);

}
