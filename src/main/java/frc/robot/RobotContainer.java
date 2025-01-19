// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Drive;
import frc.robot.controlboard.ControlBoard;
/**
 * This class is where the bulk of the robot should be declared, including subsystems, OI devices,
 * and commands. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 */
public class RobotContainer {
    private Drive driveSubsystem;
    private ControlBoard controller;

    public RobotContainer() {
        driveSubsystem = Drive.getInstance();
        controller = ControlBoard.getInstance();

        driveSubsystem.setDefaultCommand(
            new RunCommand(
                () -> driveSubsystem.setTargetSpeeds(
                    controller.getSwerveTranslation(), controller.getSwerveRotation()
                ),
                driveSubsystem
            )
        );
        // Initialization of subsystems, OI devices, and commands:
        // this.exampleSubsystem = new ExampleSubsystem();
        // this.driverController = new CommandXboxController(Ports.PIGEON);

        // The below code defines trigger->command mappings; for defining these mappings:
        //   Schedule ExampleCommand when exampleCondition becomes true:
        //     new Trigger(exampleSubsystem::exampleCondition)
        //       .onTrue(new ExampleCommand(exampleSubsystem));
        //   Schedule exampleMethodCommand when the Xbox controller's B button is pressed,
        //   cancelling on release:
        //     driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());

    }
}
