// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.*;
import frc.robot.controlboard.ControlBoard;
import frc.robot.controlboard.CustomXboxController.Button;
import frc.robot.controlboard.CustomXboxController.Side;
import frc.robot.lib.swerve.SwerveModule;

/**
 * This class is where the bulk of the robot should be declared, including subsystems, OI devices,
 * and commands. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the scheduler calls);
 * trigger->command mappings should be defined here, using the Trigger constructor
 */
public class RobotContainer {
    private ControlBoard controller = ControlBoard.getInstance();
    private Drive drive;
    private Elevator elevator;
    // private ClimberHook climberHook;
    private Coral coral;
    private Algae algae;
    private Trimmer trimmer;

    public RobotContainer() {
        // Start the Camera
        CameraServer.startAutomaticCapture();

        /* DRIVE SUBSYSTEM AND COMMANDS */
        drive = Drive.getInstance();
        SmartDashboard.putData(drive);
        for (SwerveModule sm: drive.mModules) SmartDashboard.putData("SwerveModule_" + sm.name, sm);
        drive.setDefaultCommand(
            new RunCommand(
                () -> drive.setTargetSpeeds(controller.getSwerveTranslation(), controller.getSwerveRotation()),
                drive
            )
        );
        // Right switch up
        new Trigger(() -> controller.driver.getRawButton(2)).onTrue(
            new InstantCommand(drive::zeroGyro)
        );

        /* ELEVATOR SUBSYSTEM AND COMMANDS */
        elevator = Elevator.getInstance();
        SmartDashboard.putData(elevator);
        new Trigger(() -> controller.operator.getButton(Button.LB)).onTrue(
            new InstantCommand(elevator::stepDown, elevator)
            // new FunctionalCommand(elevator::stepDown, ()->{}, (v)->{}, elevator::doneMoving, elevator)
        );
        new Trigger(() -> controller.operator.getButton(Button.RB)).onTrue(
            new InstantCommand(elevator::stepUp, elevator)
            // new FunctionalCommand(elevator::stepUp, ()->{}, (v)->{}, elevator::doneMoving, elevator)
        );
        new Trigger(() -> controller.operator.getButton(Button.START)).onTrue(
            new InstantCommand(elevator::markMin, elevator)
        );

        /* CLIMBERHOOK SUBSYSTEM AND COMMANDS */
        // climberHook = ClimberHook.getInstance();
        // SmartDashboard.putData(climberHook);
        // new Trigger(() -> controller.operator.getButton(Button.BACK)).onTrue(
        //     new InstantCommand(climberHook::toggleTarget, climberHook)
        // );

        /* CORAL SUBSYSTEM AND COMMANDS */
        coral = Coral.getInstance();
        SmartDashboard.putData(coral);
        new Trigger(() -> controller.operator.getButton(Button.X)).onTrue(
            new InstantCommand(coral::activateCoral, coral)
        );

        /* ALGAE SUBSYSTEM AND COMMANDS */
        algae = Algae.getInstance();
        SmartDashboard.putData(algae);
        new Trigger(() -> controller.operator.getButton(Button.A)).onTrue(
            new InstantCommand(() -> algae.setPosition(Algae.PositionState.DOWN), algae)
        );
        new Trigger(() -> controller.operator.getButton(Button.Y)).onTrue(
            new InstantCommand(() -> algae.setPosition(Algae.PositionState.UP), algae)
        );
        new Trigger(() -> controller.operator.getButton(Button.B)).onTrue(
            new InstantCommand(algae :: toggleDrive, algae)
        );
        new Trigger(() -> controller.operator.getTrigger(Side.RIGHT))
            .onTrue(new InstantCommand(() -> algae.nudgeDrive(-1), algae))
            .onFalse(new InstantCommand(() -> algae.nudgeDrive(0), algae));        
        new Trigger(() -> controller.operator.getTrigger(Side.LEFT))
            .onTrue(new InstantCommand(() -> algae.nudgeDrive(1), algae))
            .onFalse(new InstantCommand(() -> algae.nudgeDrive(0), algae));


        /* 
         * TRIMMER - all subsystems can add items to be adjusted 
         * These commands are marked to still run in disabled mode, so we can
         * tweak parameters and choose auto commands prior to the match starting.
        */
        trimmer = Trimmer.getInstance();
        SmartDashboard.putData(trimmer);
        new Trigger(() -> (controller.operator.getController().getPOV() == 270)).onTrue(
            new InstantCommand(trimmer::nextSubsystem, trimmer).ignoringDisable(true)
        );
        new Trigger(() -> (controller.operator.getController().getPOV() == 90)).onTrue(
            new InstantCommand(trimmer::nextItem, trimmer).ignoringDisable(true)
        );
        new Trigger(() -> (controller.operator.getController().getPOV() == 0)).onTrue(
            new InstantCommand(trimmer::incrementItem, trimmer).ignoringDisable(true)
        );
        new Trigger(() -> (controller.operator.getController().getPOV() == 180)).onTrue(
            new InstantCommand(trimmer::decrementItem, trimmer).ignoringDisable(true)
        );
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
