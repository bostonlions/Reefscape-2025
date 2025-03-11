// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.*;
import frc.robot.Constants.ElevatorConstants.Position;
import frc.robot.lib.drivers.ControlBoard;
import frc.robot.lib.drivers.CustomXboxController.Button;
import frc.robot.lib.drivers.CustomXboxController.Side;
import frc.robot.lib.drivers.CustomXboxController.Axis;
import frc.robot.lib.swerve.SwerveModule;

/**
 * This class is where the bulk of the robot should be declared, including subsystems, OI devices,
 * and commands. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the scheduler calls);
 * {@link Trigger} -> {@link Command} mappings should be defined here, using the {@link Trigger}
 * constructor for the {@link Trigger}s
 */
public final class RobotContainer {
    private final ControlBoard controller = ControlBoard.getInstance();
    private final Drive drive;
    private final Elevator elevator;
    private final ClimberHook climberHook;
    private final Coral coral;
    private final Algae algae;
    private final Trimmer trimmer;
    private final Auton auton;

    public RobotContainer() {
        // CameraServer.startAutomaticCapture(0); // Start the Camera 1
        // CameraServer.startAutomaticCapture(1); // Start the Camera 2
        // CameraServer.startAutomaticCapture(2); // Start the Camera 3
        UsbCamera cam1 = CameraServer.startAutomaticCapture(0); // Start the Camera 1
        UsbCamera cam2 = CameraServer.startAutomaticCapture(1); // Start the Camera 2
        UsbCamera cam3 = CameraServer.startAutomaticCapture(2); // Start the Camera 3
        cam1.setResolution(80,60);
        cam1.setFPS(15);
        cam2.setResolution(80,60);
        cam2.setFPS(15);
        cam3.setResolution(80,60);
        cam3.setFPS(15);

        /* DRIVE SUBSYSTEM AND COMMANDS */
        drive = Drive.getInstance();
        SmartDashboard.putData(drive);
        for (SwerveModule mod : drive.mModules) SmartDashboard.putData("SwerveModule_" + mod.name, mod);

        drive.setDefaultCommand(
            new RunCommand(
                () -> drive.setTargetSpeeds(
                    controller.getSwerveTranslation(),
                    controller.getSwerveRotation(),
                    // C switch toggles strafe mode AND snaps to reef
                    (controller.driver.getRawAxis(7) > 0.5),
                    (controller.driver.getRawAxis(6) > 0.5) // D switch toggles precision mode
                ),
                drive
            )
        );
        // C switch toggles strafe mode AND snaps to reef
        new Trigger(() -> controller.driver.getRawAxis(7) > 0.5).onTrue(drive.snapToReef());

        // zero with climber pointing towards you -- LB button
        new Trigger(() -> controller.driver.getRawButton(1)).onTrue(
            new InstantCommand(() -> drive.zeroGyro(0)).ignoringDisable(true)
        );
        // zero with algae pointing towards you -- RB button
        new Trigger(() -> controller.driver.getRawButton(2)).onTrue(
            new InstantCommand(drive::zeroGyro).ignoringDisable(true)
        );

        /* ELEVATOR SUBSYSTEM AND COMMANDS */
        elevator = Elevator.getInstance();
        SmartDashboard.putData(elevator);
        new Trigger(() -> controller.operator.getButton(Button.LB)).onTrue(elevator.stepDownCommand());
        new Trigger(() -> controller.operator.getButton(Button.RB)).onTrue(elevator.stepUpCommand());
        new Trigger(() -> controller.operator.getButton(Button.BACK)).onTrue(elevator.forceDownCommand());

        /* CLIMBERHOOK SUBSYSTEM AND COMMANDS */
        climberHook = ClimberHook.getInstance();
        SmartDashboard.putData(climberHook);
        new Trigger(() -> controller.operator.getAxis(Side.LEFT, Axis.Y) < -0.5)
            .onTrue(climberHook.nudgeUpCommand())
            .onFalse(climberHook.nudgeStopCommand());
        new Trigger(() -> controller.operator.getAxis(Side.LEFT, Axis.Y) > 0.5)
            .onTrue(climberHook.nudgeDownCommand())
            .onFalse(climberHook.nudgeStopCommand());
        // // new Trigger(() -> controller.operator.getAxis(Side.LEFT, Axis.X) > 0.5)
        //     .onTrue(climberHook.extendCommand());
        // new Trigger(() -> controller.operator.getAxis(Side.LEFT, Axis.X) < -0.5)
        //     .onTrue(climberHook.climbCommand());
        // Trigger(() -> controller.operator.getButton(Button.START)).onTrue(climberHook.footReleaseCommand());

        /* CORAL SUBSYSTEM AND COMMANDS */
        coral = Coral.getInstance();
        SmartDashboard.putData(coral);
        new Trigger(() -> controller.operator.getButton(Button.X)).onTrue(coral.toggleCommand());

        /* ALGAE SUBSYSTEM AND COMMANDS */
        algae = Algae.getInstance();
        SmartDashboard.putData(algae);
        new Trigger(() -> controller.operator.getButton(Button.A)).onTrue(algae.downCommand());
        new Trigger(() -> controller.operator.getButton(Button.Y)).onTrue(algae.upCommand());
        new Trigger(() -> controller.operator.getButton(Button.B)).onTrue(algae.toggleDriveCommand());
        new Trigger(() -> controller.operator.getAxis(Side.RIGHT, Axis.Y) < -.5)
            .onTrue(algae.nudgeRightCommand())
            .onFalse(algae.nudgeStopCommand());
        new Trigger(() -> controller.operator.getAxis(Side.RIGHT, Axis.Y) > .5)
            .onTrue(algae.nudgeLeftCommand())
            .onFalse(algae.nudgeStopCommand());
        new Trigger(() -> controller.operator.getTrigger(Side.RIGHT))
            .onTrue(elevator.stepToCommand(Position.L4));
        new Trigger(() -> controller.operator.getTrigger(Side.LEFT))
            .onTrue(elevator.stepToCommand(Position.L3));

        // new Trigger(() -> controller.operator.getTrigger(Side.LEFT))
        //     .onTrue(algae.nudgeLeftCommand())
        //     .onFalse(algae.nudgeStopCommand());

        /*
         * TRIMMER - all subsystems can add items to be adjusted
         * These commands are marked to run in disabled mode, so we can
         * tweak parameters and choose auto commands prior to the match starting.
         */
        trimmer = Trimmer.getInstance();
        SmartDashboard.putData(trimmer);
        new Trigger(() -> (controller.operator.getController().getPOV() == 270)).onTrue(trimmer.nextSubsystemCommand());
        new Trigger(() -> (controller.operator.getController().getPOV() == 90)).onTrue(trimmer.nextItemCommand());
        new Trigger(() -> (controller.operator.getController().getPOV() == 0)).onTrue(trimmer.incrementItemCommand());
        new Trigger(() -> (controller.operator.getController().getPOV() == 180)).onTrue(trimmer.decrementItemCommand());

        /* Autonomous commands */
        auton = Auton.getInstance();
        SmartDashboard.putData(auton);
    }

    public Command getAutonomousCommand() {
        return auton.getCommand();
    }

    public void teleopInit() {
        drive.setTargetSpeeds(new Translation2d(), 0, false, false);

        // climberHook.markPosition(Position.STOW);
    }
}
