// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter class to the startRobot call.
 */
public final class Main {
  private Main() {}

  /**
   * Main initialization function. Do not perform any initialization here.
   * <p>If you change your main robot class, change the parameter type.
   * 
   * Note: if deploying code isn't working, it might be because of vendordeps
   * updates, in which case the code has to be built once first to allow for
   * deploying in the future
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
