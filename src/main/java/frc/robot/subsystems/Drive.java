package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import frc.robot.lib.swerve.SwerveModule;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Ports;
import frc.robot.lib.drivers.Pigeon;
import frc.robot.lib.swerve.DriveMotionPlanner;
import frc.robot.lib.swerve.ModuleState;
import frc.robot.lib.swerve.SwerveDriveOdometry;
import frc.robot.lib.swerve.SwerveDriveKinematics;

public final class Drive extends SubsystemBase {
    public SwerveModule[] mModules;
    private Pigeon mPigeon = Pigeon.getInstance();
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private DriveControlState mControlState = DriveControlState.FORCE_ORIENT;
    private final SwerveDriveOdometry mOdometry;
    private final DriveMotionPlanner mMotionPlanner;
    private SwerveConstants.KinematicLimits mKinematicLimits = SwerveConstants.kUncappedLimits;
    private SwerveDriveKinematics kKinematics;
    private static Drive mInstance;
    public enum DriveControlState { FORCE_ORIENT, OPEN_LOOP, PATH_FOLLOWING }

    public static Drive getInstance() {
        if (mInstance == null) mInstance = new Drive();
        return mInstance;
    }

    private Drive() {
        mModules = new SwerveModule[] {
            new SwerveModule(0, new SwerveModule.Constants(
                Ports.FR_DRIVE, Ports.FR_ROTATION, Ports.FR_CANCODER, SwerveConstants.FR_AngleOffset, "FR"
            )),
            new SwerveModule(1, new SwerveModule.Constants(
                Ports.FL_DRIVE, Ports.FL_ROTATION, Ports.FL_CANCODER, SwerveConstants.FL_AngleOffset, "FL"
            )),
            new SwerveModule(2, new SwerveModule.Constants(
                Ports.BR_DRIVE, Ports.BR_ROTATION, Ports.BR_CANCODER, SwerveConstants.BR_AngleOffset, "BR"
            )),
            new SwerveModule(3, new SwerveModule.Constants(
                Ports.BL_DRIVE, Ports.BL_ROTATION, Ports.BL_CANCODER, SwerveConstants.BL_AngleOffset, "BL"
            ))
        };

        kKinematics = new SwerveDriveKinematics(SwerveConstants.wheelBase, SwerveConstants.trackWidth);
        mOdometry = new SwerveDriveOdometry(kKinematics, getModuleStates());
        mMotionPlanner = new DriveMotionPlanner();

        setNeutralBrake(true);
        mPigeon.setYaw(0.0);
    }

    // private void setKinematicLimits(SwerveConstants.KinematicLimits newLimits) {
    //     mKinematicLimits = newLimits;
    // }

    private void feedTeleopSetpoint(ChassisSpeeds speeds) {
        if (mControlState != DriveControlState.OPEN_LOOP) mControlState = DriveControlState.OPEN_LOOP;
        mPeriodicIO.des_chassis_speeds = speeds;
    }

    public void setTargetSpeeds(Translation2d targetSpeed, double targetRotationRate, boolean strafe) {
        ChassisSpeeds speeds;
        mPeriodicIO.strafeMode = strafe;
        if (strafe) {
            speeds = new ChassisSpeeds( //ChassisSpeeds.fromRobotRelativeSpeeds(
                targetSpeed.getX(),
                targetSpeed.getY(),
                targetRotationRate
            );
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                targetSpeed.getX(),
                targetSpeed.getY(),
                targetRotationRate,
                getHeading()
            );
        }
        feedTeleopSetpoint(speeds);
    }

    // private void setHeadingControlTarget(double target_degrees) {
    //     if (mControlState != DriveControlState.HEADING_CONTROL) mControlState = DriveControlState.HEADING_CONTROL;
    //     mPeriodicIO.heading_setpoint = Rotation2d.fromDegrees(target_degrees);
    // }

    // private void setOpenLoop(ChassisSpeeds speeds) {
    //     mPeriodicIO.des_chassis_speeds = speeds;
    //     if (mControlState != DriveControlState.OPEN_LOOP) mControlState = DriveControlState.OPEN_LOOP;
    // }

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
                    (pose) -> { System.out.println("resetOdometry " + pose); resetOdometry(pose); }
                );
            }).andThen(new FollowPathCommand(
                path,
                () -> {
                    //System.out.println("getPose: " + this.getPose());
                    return this.getPose();
                },
                () -> {
                    //System.out.println("getSpeeds " + mPeriodicIO.meas_chassis_speeds);
                    return mPeriodicIO.meas_chassis_speeds; // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                },
                (ChassisSpeeds setPointSpeeds, DriveFeedforwards dff/*unused; not needed*/) -> {
                    mControlState = DriveControlState.PATH_FOLLOWING;
                    mPeriodicIO.des_chassis_speeds = setPointSpeeds;
                    updateSetpoint();
                    //System.out.println("path following " + setPointSpeeds + ". At " + (System.currentTimeMillis() % 60000) + "ms after the start of this minute");
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

    /** Stops drive without orienting modules */
    private synchronized void stopModules() {
        List<Rotation2d> orientations = new ArrayList<>();
        for (ModuleState moduleState: getModuleStates()) orientations.add(moduleState.angle);
        orientModules(orientations);
    }

    private synchronized void orientModules(List<Rotation2d> orientations) {
        if (mControlState != DriveControlState.FORCE_ORIENT) mControlState = DriveControlState.FORCE_ORIENT;
        for (int i = 0; i < mModules.length; ++i) mPeriodicIO.des_module_states[i] =
            ModuleState.fromSpeeds(orientations.get(i), 0.0);
    }

    private void updateSetpoint() {
        if (mControlState == DriveControlState.FORCE_ORIENT) return;

        Pose2d robot_pose_vel = new Pose2d(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * SwerveConstants.kLooperDt,
            mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * SwerveConstants.kLooperDt,
            Rotation2d.fromRadians(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * SwerveConstants.kLooperDt));
        Twist2d twist_vel = new Pose2d().log(robot_pose_vel);
        ChassisSpeeds wanted_speeds = new ChassisSpeeds(
            twist_vel.dx / SwerveConstants.kLooperDt, twist_vel.dy / SwerveConstants.kLooperDt,
            twist_vel.dtheta / SwerveConstants.kLooperDt);

        if (mControlState == DriveControlState.PATH_FOLLOWING) {
            ModuleState[] real_module_setpoints = kKinematics.toModuleStates(wanted_speeds);
            mPeriodicIO.des_module_states = real_module_setpoints;
            return;
        }

        // Limit rotational velocity
        wanted_speeds.omegaRadiansPerSecond = Math.signum(wanted_speeds.omegaRadiansPerSecond)
            * Math.min(mKinematicLimits.kMaxAngularVelocity, Math.abs(wanted_speeds.omegaRadiansPerSecond));

        // Limit translational velocity
        double velocity_magnitude = Math.hypot(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond,
            mPeriodicIO.des_chassis_speeds.vyMetersPerSecond);
        if (velocity_magnitude > mKinematicLimits.kMaxDriveVelocity) {
            wanted_speeds.vxMetersPerSecond = (wanted_speeds.vxMetersPerSecond / velocity_magnitude)
                * mKinematicLimits.kMaxDriveVelocity;
            wanted_speeds.vyMetersPerSecond = (wanted_speeds.vyMetersPerSecond / velocity_magnitude)
                * mKinematicLimits.kMaxDriveVelocity;
        }

        ModuleState[] prev_module_states = mPeriodicIO.des_module_states.clone(); // Get last setpoint to get differentials
        ChassisSpeeds prev_chassis_speeds = kKinematics.toChassisSpeeds(prev_module_states);

        double dx = wanted_speeds.vxMetersPerSecond - prev_chassis_speeds.vxMetersPerSecond;
        double dy = wanted_speeds.vyMetersPerSecond - prev_chassis_speeds.vyMetersPerSecond;
        double domega = wanted_speeds.omegaRadiansPerSecond - prev_chassis_speeds.omegaRadiansPerSecond;

        double max_velocity_step = mKinematicLimits.kMaxAccel * SwerveConstants.kLooperDt;
        double min_translational_scalar = 1.;

        if (max_velocity_step < Double.MAX_VALUE * SwerveConstants.kLooperDt) {
            // Check X
            double x_norm = Math.abs(dx / max_velocity_step);
            min_translational_scalar = Math.min(min_translational_scalar, x_norm);

            // Check Y
            double y_norm = Math.abs(dy / max_velocity_step);
            min_translational_scalar = Math.min(min_translational_scalar, y_norm);

            min_translational_scalar *= max_velocity_step;
        }

        double max_omega_step = mKinematicLimits.kMaxAngularAccel * SwerveConstants.kLooperDt;
        double min_omega_scalar = 1.0;

        if (max_omega_step < Double.MAX_VALUE * SwerveConstants.kLooperDt) {
            double omega_norm = Math.abs(domega / max_omega_step);
            min_omega_scalar = Math.min(min_omega_scalar, omega_norm);
            min_omega_scalar *= max_omega_step;
        }

        double finalVx = prev_chassis_speeds.vxMetersPerSecond + dx * min_translational_scalar;
        double finalVy = prev_chassis_speeds.vyMetersPerSecond + dy * min_translational_scalar;
        double finalOmega = prev_chassis_speeds.omegaRadiansPerSecond + domega * min_omega_scalar;

        wanted_speeds = new ChassisSpeeds(finalVx, finalVy, finalOmega);
        ModuleState[] real_module_setpoints = kKinematics.toModuleStates(wanted_speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(real_module_setpoints, SwerveConstants.maxAttainableSpeed);

        // Ensure we don't alter angles if we're shooting for zero speed
        if (
            MathUtil.isNear(finalVx, 0, 1e-12) &&
            MathUtil.isNear(finalVy, 0, 1e-12) &&
            MathUtil.isNear(finalOmega, 0, 1e-12)
        ) {
            for (int i = 0; i < real_module_setpoints.length; i++) {
                real_module_setpoints[i].speedMetersPerSecond = 0.;
                real_module_setpoints[i].angle = prev_module_states[i].angle;
            }
        }

        mPeriodicIO.des_module_states = real_module_setpoints;
    }

    public void zeroGyro() {
        zeroGyro(180.);
    }

    private void zeroGyro(double reset) {
        mPigeon.setYaw(reset);
        mPigeon.setPitch(0.);
    }

    private void setNeutralBrake(boolean brake) {
        for (SwerveModule mod: mModules) mod.setDriveNeutralBrake(brake);
    }

    @Override
    public void periodic() {
        /* read periodic inputs */
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.meas_module_states = getModuleStates();
        mPeriodicIO.meas_chassis_speeds = kKinematics.toChassisSpeeds(mPeriodicIO.meas_module_states);
        mPeriodicIO.heading = mPigeon.getYaw();
        mPeriodicIO.pitch = mPigeon.getPitch();

        // Update odometry so we track where we are on the field
        mOdometry.update(mPeriodicIO.heading, getModuleStates());

        updateSetpoint();

        for (SwerveModule mod : mModules) mod.setDesiredState(mPeriodicIO.des_module_states
            [mod.moduleNumber()], mControlState == DriveControlState.OPEN_LOOP);

        for (SwerveModule mod: mModules) mod.periodic(); // read and write module periodic io
    }

    private ModuleState[] getModuleStates() {
        ModuleState[] states = new ModuleState[4];
        for (SwerveModule mod: mModules) states[mod.moduleNumber()] = mod.getState();
        return states;
    }

    private Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    private void resetOdometry(Pose2d pose) {
        Pose2d wanted_pose = pose;
        Rotation2d wantedRotationReset = pose.getRotation();
        // may need to bring back later:
        // if (Robot.flip_trajectories)  {
        //     wantedRotationReset.rotateBy(Rotation2d.fromDegrees(180));
        // }
        mOdometry.resetPosition(getModuleStates(), wanted_pose);
        zeroGyro(wantedRotationReset.getDegrees());
    }

    // public boolean isDoneWithTrajectory() {
    //     if (mControlState != DriveControlState.PATH_FOLLOWING) return false;
    //     return mMotionPlanner.isFinished();
    // }

    private Rotation2d getHeading() {
        return mPigeon.getYaw();
    }

    public SwerveConstants.KinematicLimits getKinematicLimits() {
        return mKinematicLimits;
    }

    private static final class PeriodicIO {
        // Inputs/Desired States
        double timestamp;
        ChassisSpeeds des_chassis_speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        ChassisSpeeds meas_chassis_speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        ModuleState[] meas_module_states = new ModuleState[] {
            new ModuleState(),
            new ModuleState(),
            new ModuleState(),
            new ModuleState()
        };
        Rotation2d heading = new Rotation2d();
        Rotation2d pitch = new Rotation2d();
        boolean strafeMode = false;

        // Outputs
        ModuleState[] des_module_states = new ModuleState[] {
            new ModuleState(),
            new ModuleState(),
            new ModuleState(),
            new ModuleState()
        };
    }

    // private void setConfigs() { // TODO: make trimmer for drive (which will use this)
    //     for (SwerveModule mod: mModules) mod.setConfigs();
    // }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Drive");
        builder.setSafeState(this::stopModules);
        builder.setActuator(true);

        builder.addDoubleProperty("Pitch", () -> mPeriodicIO.pitch.getDegrees(), null);
        builder.addStringProperty("Drive Control State", () -> mControlState.toString(), null);
        builder.addBooleanProperty("Strafe Mode", () -> mPeriodicIO.strafeMode, null);
        builder.addDoubleProperty("ROBOT HEADING", () -> getHeading().getDegrees(), null);
        builder.addDoubleProperty("Timestamp", () -> mPeriodicIO.timestamp, null);
        builder.addDoubleProperty("Trajectory X Error", () -> mMotionPlanner.getXError(getPose().getX(), Timer.getFPGATimestamp()), null);
        builder.addDoubleProperty("Trajectory Y Error", () -> mMotionPlanner.getYError(getPose().getY(), Timer.getFPGATimestamp()), null);
        builder.addDoubleProperty("Rotation Error", () -> mMotionPlanner.getRotationalError(getPose().getRotation().getDegrees()), null);
        builder.addDoubleProperty("Trajectory X", () -> mMotionPlanner.getXError(0.0, Timer.getFPGATimestamp()), null);
        builder.addDoubleProperty("Trajectory Y", () -> mMotionPlanner.getYError(0.0, Timer.getFPGATimestamp()), null);
        builder.addDoubleProperty("Trajectory Heading", mMotionPlanner::getRotationalTarget, null);
        builder.addDoubleProperty("Measured Velocity X", () -> mPeriodicIO.meas_chassis_speeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Measured Velocity Y", () -> mPeriodicIO.meas_chassis_speeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("Measured Omega rad_s", () -> mPeriodicIO.meas_chassis_speeds.omegaRadiansPerSecond, null);
        builder.addDoubleProperty("Target Velocity X", () -> mPeriodicIO.des_chassis_speeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Target Velocity Y", () -> mPeriodicIO.des_chassis_speeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("Target Omega rad_s", () -> mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond, null);
        builder.addDoubleProperty("Pose X", () -> getPose().getX(), null);
        builder.addDoubleProperty("Pose Y", () -> getPose().getY(), null);
        builder.addDoubleProperty("Pose Theta", () -> getPose().getRotation().getDegrees(), null);
    }
}
