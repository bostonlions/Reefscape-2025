// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

final class Main {
    public static void main(String... args) { // if deploying code isn't working, it might be because of
        RobotBase.startRobot(Robot::new);     // vendordeps updates, in which case the code has to be
    }                                         // built once first to allow for deploying in the future
}
