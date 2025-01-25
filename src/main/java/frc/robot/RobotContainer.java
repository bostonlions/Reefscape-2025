// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.CustomXboxController.Button;

/**
 * This class is where the bulk of the robot should be declared, including subsystems, OI devices,
 * and commands. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the scheduler calls);
 * Trigger->command mappings should be defined here; for defining these mappings:
 *      Schedule ExampleCommand when exampleCondition becomes true:
 *          new Trigger(exampleSubsystem::exampleCondition)
 *            .onTrue(new ExampleCommand(exampleSubsystem));
 *      Schedule exampleMethodCommand when the Xbox controller's B button is pressed,
 *      cancelling on release:
 *          driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
 */
public class RobotContainer {
    private ControlBoard controller;
    private Drive drive;
    private Elevator elevator;

    public RobotContainer() {
        controller = ControlBoard.getInstance();

        /* DRIVE SUBSYSTEM AND COMMANDS */
        drive = Drive.getInstance();
        SmartDashboard.putData(drive);
        for (int i = 0; i<=3; i++) {
            SmartDashboard.putData(drive.mModules[i]);
        }
        drive.setDefaultCommand(
            new RunCommand(
                () -> drive.setTargetSpeeds(controller.getSwerveTranslation(), controller.getSwerveRotation()),
                drive
            )
        );

        /* ELEVATOR SUBSYSTEM AND COMMANDS */
        elevator = Elevator.getInstance();
        SmartDashboard.putData(elevator);
        new Trigger(() -> controller.operator.getButton(Button.RB)).onTrue(
            new FunctionalCommand(elevator::stepDown, null, null, elevator::doneMoving, elevator)
        );
        new Trigger(() -> controller.operator.getButton(Button.LB)).onTrue(
            new FunctionalCommand(elevator::stepUp, null, null, elevator::doneMoving, elevator)
        );
        new Trigger(() -> controller.operator.getButton(Button.START)).onTrue(
            new InstantCommand(elevator::markMin, elevator)
        );
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
