// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
    /**
     * If this isn't working to deploy code, it might be because of vendordeps updates, in
     * which case the code has to be built once first to allow for deploying in the future
     */
    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
