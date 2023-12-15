// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LimelightPose.h"

LimelightPose::LimelightPose() = default;

// This method will be called once per scheduler run
void LimelightPose::Periodic() {


    botPose = frc::SmartDashboard::GetNumberArray("botpose", std::span<const double>({0, 0, 0, 0, 0, 0}));
    
    std::cout << botPose[0] << std::endl;
    std::cout << botPose[1] << std::endl;
    std::cout << botPose[2] << std::endl;
    std::cout << "end" << std::endl;
}
