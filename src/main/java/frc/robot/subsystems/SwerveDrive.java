package frc.robot.subsystems;

import java.util.List;
import java.util.ArrayList;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.Pair;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.SwerveConstants;

public final class SwerveDrive extends SubsystemBase {
    private static SwerveDrive instance;
    private SwerveDrivetrain<TalonFX, TalonFX, CANcoder> driveTrain;
    private boolean strafeMode;
    private ChassisSpeeds requestedSpeeds = new ChassisSpeeds();
    /** Robot relative */
    public List<Pair<Double, Double>> swerveModulePositions = new ArrayList<>();

    public static SwerveDrive getInstance() {
        if (instance == null) instance = new SwerveDrive();
        return instance;
    }

    private SwerveDrive() {
        driveTrain = new SwerveDrivetrain<TalonFX, TalonFX, CANcoder>(
            TalonFX::new, TalonFX::new, CANcoder::new,
            new SwerveDrivetrainConstants()
                .withCANBusName(Ports.CANBUS_DRIVE)
                .withPigeon2Id(Ports.PIGEON)
                .withPigeon2Configs(new Pigeon2Configuration()),
            getNewSMConstants(
                Ports.FR_ROTATION, Ports.FR_DRIVE, Ports.FR_CANCODER, SwerveConstants.FR_AngleOffset,
                (byte) 1, (byte) 1
            ),
            getNewSMConstants(
                Ports.FL_ROTATION, Ports.FL_DRIVE, Ports.FL_CANCODER, SwerveConstants.FL_AngleOffset,
                (byte) -1, (byte) 1
            ),
            getNewSMConstants(
                Ports.BR_ROTATION, Ports.BR_DRIVE, Ports.BR_CANCODER, SwerveConstants.BR_AngleOffset,
                (byte) 1, (byte) -1
            ),
            getNewSMConstants(
                Ports.BL_ROTATION, Ports.BL_DRIVE, Ports.BL_CANCODER, SwerveConstants.BL_AngleOffset,
                (byte) -1, (byte) -1
            )
        );
    }

    private SwerveModuleConstants<
        TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration
    > getNewSMConstants(
        int anglePort, int drivePort, int CANPort, double CANcoderOffset, byte xSign, byte ySign
    ) {
        swerveModulePositions.add(new Pair<Double, Double>(
            xSign * SwerveConstants.trackWidth / 2, ySign * SwerveConstants.wheelBase / 2
        ));

        return new SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration
        >().createModuleConstants(
            anglePort, drivePort, CANPort, CANcoderOffset,
            swerveModulePositions.get(swerveModulePositions.size() - 1).getFirst(),
            swerveModulePositions.get(swerveModulePositions.size() - 1).getSecond(),
            false, false, false
        )
        .withCouplingGearRatio(SwerveConstants.couplingGearRatio)
        .withDriveInertia(3.873) // TODO: get this right?
        .withDriveMotorGains(SwerveConstants.driveConfig.Slot0)
        .withDriveMotorGearRatio(SwerveConstants.driveGearRatio)
        .withDriveMotorInitialConfigs(SwerveConstants.driveConfig)
        .withEncoderInitialConfigs(SwerveConstants.cancoderConfig)
        .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
        .withSteerMotorGains(SwerveConstants.angleConfig.Slot0)
        .withSteerMotorGearRatio(SwerveConstants.angleGearRatio)
        .withSteerMotorInitialConfigs(SwerveConstants.angleConfig)
        .withWheelRadius(SwerveConstants.wheelDiameter / 2);
    }

    public Command followPathCommand(String pathName, boolean isFirstPath) {
        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            // /* TO DEBUG */
            // System.out.println(path.getWaypoints());
            // System.out.println(path.getAllPathPoints().stream().map(e->e.position).toList());
            // System.out.println(List.of(AutonConstants.moduleTranslations));
            // PathPlannerTrajectory traj = path.generateTrajectory(mPeriodicIO.meas_chassis_speeds, getHeading(), AutonConstants.pathPlannerConfig);
            // System.out.println(pathName + " trajectory time: " + traj.getTotalTimeSeconds() + ", state speeds: " + traj.getStates().stream().map(e->e.fieldSpeeds).toList());
            // System.out.println(pathName + " trajectory time: " + traj.getTotalTimeSeconds() + ", state poses: " + traj.getStates().stream().map(e->e.pose).toList());

            return new InstantCommand(() -> {
                if (isFirstPath) path.getStartingHolonomicPose().ifPresent(
                    (pose) -> { System.out.println("resetOdometry " + pose); driveTrain.resetPose(pose); }
                );
            }).andThen(new FollowPathCommand(
                path,
                () -> driveTrain.getState().Pose,
                () -> {
                    // TODO: if driveTrain.getState().Speeds is field-relative, use this one:
                    // return ChassisSpeeds.fromFieldRelativeSpeeds(driveTrain.getState().Speeds, driveTrain.getState().Pose.getRotation());
                    // TODO part 2: and if it's robot-relative, use this one:
                    return driveTrain.getState().Speeds; // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                },
                (ChassisSpeeds setPointSpeeds, DriveFeedforwards dff) -> {
                    driveTrain.setControl(
                        new SwerveRequest.ApplyRobotSpeeds()
                            .withSpeeds(setPointSpeeds)
                            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                            .withWheelForceFeedforwardsX(dff.robotRelativeForcesX())
                            .withWheelForceFeedforwardsY(dff.robotRelativeForcesY())
                    );
                    // System.out.println("path following " + setPointSpeeds + ". At " + (System.currentTimeMillis() % 60000) + "ms after the start of this minute");
                }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                AutonConstants.ppHolonomicDriveController,
                AutonConstants.pathPlannerConfig,
                () -> false,
                this
            ));
        } catch (Exception e) {
            System.out.println(
                "\n------------------------------------------------\n" +
                "Big oops: " + e.getMessage() + "\n\n" + e.getStackTrace() +
                "\n------------------------------------------------\n"
            );
            return Commands.none();
        }
    }

    public void zeroGyro() {
        driveTrain.resetRotation(new Rotation2d(180.));
    }

    public void setTargetSpeeds(Translation2d targetSpeed, double targetRotationRate, boolean strafe) {
        strafeMode = strafe;
        requestedSpeeds = new ChassisSpeeds(targetSpeed.getX(), targetSpeed.getY(), targetRotationRate);
        
        if (strafe) {
            driveTrain.setControl(
                new SwerveRequest.ApplyRobotSpeeds()
                    .withSpeeds(requestedSpeeds)
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            );
        } else {
            driveTrain.setControl(
                new SwerveRequest.ApplyFieldSpeeds()
                    .withSpeeds(requestedSpeeds)
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            );
        }
    }

    private void disable() {
        // TODO
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Drive");
        builder.setSafeState(this::disable);
        builder.setActuator(true);

        builder.addBooleanProperty("Strafe Mode? ", () -> strafeMode, null);
        builder.addDoubleProperty("Target X Speed (m_s)", () -> requestedSpeeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Target Y Speed (m_s)", () -> requestedSpeeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("Target Rotation Rate (rad_s)", () -> requestedSpeeds.omegaRadiansPerSecond, null);
        builder.addDoubleProperty("Measured X", () -> driveTrain.getState().Pose.getX(), null);
        builder.addDoubleProperty("Measured Y", () -> driveTrain.getState().Pose.getY(), null);
        builder.addDoubleProperty("Heading", () -> driveTrain.getState().Pose.getRotation().getDegrees(), null);
        builder.addDoubleProperty("Measured X speed (m_s)", () -> driveTrain.getState().Speeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Measured Y speed (m_s)", () -> driveTrain.getState().Speeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("Measured Rotation rate (rad_s)", () -> driveTrain.getState().Speeds.omegaRadiansPerSecond, null);

        builder.addDoubleProperty("FR angle voltage", () -> driveTrain.getModules()[0].getSteerMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("FR drive voltage", () -> driveTrain.getModules()[0].getDriveMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("FR cancoder", () -> driveTrain.getModules()[0].getEncoder().getAbsolutePosition().getValueAsDouble(), null);
        builder.addDoubleProperty("FL angle voltage", () -> driveTrain.getModules()[1].getSteerMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("FL drive voltage", () -> driveTrain.getModules()[1].getDriveMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("FL cancoder", () -> driveTrain.getModules()[1].getEncoder().getAbsolutePosition().getValueAsDouble(), null);
        builder.addDoubleProperty("BR angle voltage", () -> driveTrain.getModules()[2].getSteerMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("BR drive voltage", () -> driveTrain.getModules()[2].getDriveMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("BR cancoder", () -> driveTrain.getModules()[2].getEncoder().getAbsolutePosition().getValueAsDouble(), null);
        builder.addDoubleProperty("BL angle voltage", () -> driveTrain.getModules()[3].getSteerMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("BL drive voltage", () -> driveTrain.getModules()[3].getDriveMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("BL cancoder", () -> driveTrain.getModules()[3].getEncoder().getAbsolutePosition().getValueAsDouble(), null);
    }

    private void initTrimmer() { // TODO (will use this trimmer for PID - will use Steve's PID finding method with this)
        Trimmer trimmer = Trimmer.getInstance();
    }
}
