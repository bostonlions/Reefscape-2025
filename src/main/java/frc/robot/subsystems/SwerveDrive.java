package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Ports;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.drivers.CustomXboxController.Axis;

public final class SwerveDrive extends SubsystemBase {
    private static SwerveDrive mInstance;
    private static final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final PeriodicIO mPeriodicIO = new PeriodicIO();
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> mDriveTrain = new SwerveDrivetrain<
        TalonFX, TalonFX, CANcoder
    >(
        TalonFX::new, TalonFX::new, CANcoder::new,
        new SwerveDrivetrainConstants().withCANBusName(Ports.CANBUS_DRIVE).withPigeon2Id(Ports.PIGEON),
        SwerveConstants.SMConstFactory.createModuleConstants(
            Ports.FR_ROTATION, Ports.FR_DRIVE, Ports.FR_CANCODER, SwerveConstants.FR_AngleOffset,
            getSwerveModulePos(1, Axis.X),
            getSwerveModulePos(1, Axis.Y),
            false, false, false
        ),
        SwerveConstants.SMConstFactory.createModuleConstants(
            Ports.FL_ROTATION, Ports.FL_DRIVE, Ports.FL_CANCODER, SwerveConstants.FL_AngleOffset,
            getSwerveModulePos(2, Axis.X),
            getSwerveModulePos(2, Axis.Y),
            false, false, false
        ),
        SwerveConstants.SMConstFactory.createModuleConstants(
            Ports.BR_ROTATION, Ports.BR_DRIVE, Ports.BR_CANCODER, SwerveConstants.BR_AngleOffset,
            getSwerveModulePos(3, Axis.X),
            getSwerveModulePos(3, Axis.Y),
            false, false, false
        ),
        SwerveConstants.SMConstFactory.createModuleConstants(
            Ports.BL_ROTATION, Ports.BL_DRIVE, Ports.BL_CANCODER, SwerveConstants.BL_AngleOffset,
            getSwerveModulePos(4, Axis.X),
            getSwerveModulePos(4, Axis.Y),
            false, false, false
        )
    );

    public static SwerveDrive getInstance() {
        if (mInstance == null) mInstance = new SwerveDrive();
        return mInstance;
    }

    private SwerveDrive() {
        mDriveTrain.getOdometryThread().start(); // TODO: do we want this?
    }

    /** @return the requested coordinate of the robot-relative position of the requested swerve module */
    public static double getSwerveModulePos(int swerveModuleNum, Axis axis) {
        return (
            axis == Axis.X ? List.of(
                SwerveConstants.trackWidth / 2, -SwerveConstants.trackWidth / 2,
                SwerveConstants.trackWidth / 2, -SwerveConstants.trackWidth / 2
            ) : List.of(
                SwerveConstants.wheelBase / 2, SwerveConstants.wheelBase / 2,
                -SwerveConstants.wheelBase / 2, -SwerveConstants.wheelBase / 2
            )
        ).get(swerveModuleNum - 1);
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
                    (pose) -> mDriveTrain.resetPose(pose)
                );
            }).andThen(new FollowPathCommand(
                path,
                () -> mDriveTrain.getState().Pose,
                () -> {
                    // TODO: if driveTrain.getState().Speeds is field-relative, use this one:
                    // return ChassisSpeeds.fromFieldRelativeSpeeds(driveTrain.getState().Speeds, driveTrain.getState().Pose.getRotation());
                    // TODO part 2: and if it's robot-relative, use this one:
                    return mDriveTrain.getState().Speeds; // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                },
                (ChassisSpeeds setPointSpeeds, DriveFeedforwards dff) -> {
                    mDriveTrain.setControl(
                        new SwerveRequest.ApplyRobotSpeeds()
                            .withSpeeds(setPointSpeeds)
                            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
                            .withWheelForceFeedforwardsX(dff.robotRelativeForcesX())
                            .withWheelForceFeedforwardsY(dff.robotRelativeForcesY())
                    );
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
        mDriveTrain.resetRotation(new Rotation2d(180.));
    }

    public void zeroGyroReversed() {
        mDriveTrain.resetRotation(new Rotation2d(0.));
    }

    public void setTargetSpeeds(Translation2d targetSpeed, double targetRotationRate, boolean strafeAndSnap, boolean precisionMode) { // TODO: precision mode and strafe mode reductions; and snap implementation
        mPeriodicIO.requestedSpeeds = new ChassisSpeeds(targetSpeed.getX(), targetSpeed.getY(), targetRotationRate);
        mPeriodicIO.strafeMode = strafeAndSnap;
        mPeriodicIO.precisionMode = precisionMode;

        mDriveTrain.setControl(
            mPeriodicIO.strafeMode ? new SwerveRequest.ApplyRobotSpeeds()
                .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
                .withSteerRequestType(SteerRequestType.MotionMagicExpo)
                .withSpeeds(mPeriodicIO.requestedSpeeds)
            : fieldCentricRequest.withVelocityX(-targetSpeed.getY()).withVelocityY(-targetSpeed.getX())
                .withRotationalRate(targetRotationRate)
        );
    }

    private static final class PeriodicIO {
        private ChassisSpeeds requestedSpeeds = new ChassisSpeeds();
        private boolean strafeMode;
        private boolean precisionMode;
    }

    // @Override
    // public void periodic() {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Swerve Drive");
        builder.setActuator(true);

        builder.addBooleanProperty("Strafe mode?", () -> mPeriodicIO.strafeMode, null);
        builder.addBooleanProperty("Precision mode?", () -> mPeriodicIO.precisionMode, null);
        builder.addDoubleProperty("Target X Speed (m_s)", () -> mPeriodicIO.requestedSpeeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Target Y Speed (m_s)", () -> mPeriodicIO.requestedSpeeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("Target Rotation Rate (rad_s)", () -> mPeriodicIO.requestedSpeeds.omegaRadiansPerSecond, null);
        builder.addDoubleProperty("Measured X", () -> mDriveTrain.getState().Pose.getX(), null);
        builder.addDoubleProperty("Measured Y", () -> mDriveTrain.getState().Pose.getY(), null);
        builder.addDoubleProperty("Heading", () -> mDriveTrain.getState().Pose.getRotation().getDegrees(), null);
        builder.addDoubleProperty("Measured X speed (m_s)", () -> mDriveTrain.getState().Speeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Measured Y speed (m_s)", () -> mDriveTrain.getState().Speeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("Measured Rotation rate (rad_s)", () -> mDriveTrain.getState().Speeds.omegaRadiansPerSecond, null);

        builder.addDoubleProperty("FR angle voltage", () -> mDriveTrain.getModules()[0].getSteerMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("FR drive voltage", () -> mDriveTrain.getModules()[0].getDriveMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("FR cancoder", () -> mDriveTrain.getModules()[0].getEncoder().getAbsolutePosition().getValueAsDouble(), null);
        builder.addDoubleProperty("FL angle voltage", () -> mDriveTrain.getModules()[1].getSteerMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("FL drive voltage", () -> mDriveTrain.getModules()[1].getDriveMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("FL cancoder", () -> mDriveTrain.getModules()[1].getEncoder().getAbsolutePosition().getValueAsDouble(), null);
        builder.addDoubleProperty("BR angle voltage", () -> mDriveTrain.getModules()[2].getSteerMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("BR drive voltage", () -> mDriveTrain.getModules()[2].getDriveMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("BR cancoder", () -> mDriveTrain.getModules()[2].getEncoder().getAbsolutePosition().getValueAsDouble(), null);
        builder.addDoubleProperty("BL angle voltage", () -> mDriveTrain.getModules()[3].getSteerMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("BL drive voltage", () -> mDriveTrain.getModules()[3].getDriveMotor().getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("BL cancoder", () -> mDriveTrain.getModules()[3].getEncoder().getAbsolutePosition().getValueAsDouble(), null);
    }

    private void initTrimmer() { // TODO (will use this trimmer for PID - will use Steve's PID finding method with this)
        Trimmer trimmer = Trimmer.getInstance();
    }
}
