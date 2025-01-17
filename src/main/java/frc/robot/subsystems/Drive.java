package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.lib.swerve.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.loops.ILooper;
import frc.robot.lib.loops.Loop;
import frc.robot.lib.Util;
import frc.robot.lib.Util.MovingAverage;
import frc.robot.lib.drivers.Pigeon;
import frc.robot.lib.logger.Log;
import frc.robot.lib.logger.LoggingSystem;
import frc.robot.lib.swerve.ChassisSpeeds;
import frc.robot.lib.swerve.DriveMotionPlanner;
import frc.robot.lib.swerve.ModuleState;
import frc.robot.lib.swerve.SwerveDriveOdometry;
import frc.robot.lib.swerve.SwerveDriveKinematics;

public class Drive extends Subsystem {
    public enum DriveControlState {
        FORCE_ORIENT,
        OPEN_LOOP,
        HEADING_CONTROL,
        VELOCITY,
        PATH_FOLLOWING,
        AUTO_BALANCE
    }

    private Pigeon mPigeon = Pigeon.getInstance();
    public SwerveModule[] mModules;

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private DriveControlState mControlState = DriveControlState.FORCE_ORIENT;

    private final SwerveDriveOdometry mOdometry;
    private boolean odometryReset = false;
    private final DriveMotionPlanner mMotionPlanner;

    private SwerveConstants.KinematicLimits mKinematicLimits = SwerveConstants.kUncappedLimits;
    private SwerveDriveKinematics kKinematics;

    private static Drive mInstance;

    private boolean spinFastDuringAuto = false;

    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }
        return mInstance;
    }

    private Drive() {
        mModules = new SwerveModule[] {
            new SwerveModule(0, SwerveConstants.Mod0),
            new SwerveModule(1, SwerveConstants.Mod1),
            new SwerveModule(2, SwerveConstants.Mod2),
            new SwerveModule(3, SwerveConstants.Mod3)
        };

        kKinematics = new SwerveDriveKinematics(SwerveConstants.locations);

        mOdometry = new SwerveDriveOdometry(kKinematics, getModuleStates());
        mMotionPlanner = new DriveMotionPlanner();

        mPigeon.setYaw(0.0);

        LoggingSystem.getInstance().registerObject(SwerveModule.class, mModules[0], "MOD_0");
        LoggingSystem.getInstance().registerObject(SwerveModule.class, mModules[1], "MOD_1");
        LoggingSystem.getInstance().registerObject(SwerveModule.class, mModules[2], "MOD_2");
        LoggingSystem.getInstance().registerObject(SwerveModule.class, mModules[3], "MOD_3");
    }

    public void setKinematicLimits(SwerveConstants.KinematicLimits newLimits) {
        this.mKinematicLimits = newLimits;
    }

    public void feedTeleopSetpoint(ChassisSpeeds speeds) {
        if (mControlState != DriveControlState.OPEN_LOOP && mControlState != DriveControlState.HEADING_CONTROL) {
            mControlState = DriveControlState.OPEN_LOOP;
        }

        if (mControlState == DriveControlState.HEADING_CONTROL) {
            if (Math.abs(speeds.omegaRadiansPerSecond) > 1.0) {
                mControlState = DriveControlState.OPEN_LOOP;
            } else {
                double x = speeds.vxMetersPerSecond;
                double y = speeds.vyMetersPerSecond;
                double omega = mMotionPlanner.calculateRotationalAdjustment(mPeriodicIO.heading_setpoint.getRadians(),
                        mPeriodicIO.heading.getRadians()); // I put a neg sign here fyi
                mPeriodicIO.des_chassis_speeds = new ChassisSpeeds(x, y, omega);
                return;
            }
        }
        mPeriodicIO.des_chassis_speeds = speeds;
    }

    public void setHeadingControlTarget(double target_degrees) {
        if (mControlState != DriveControlState.HEADING_CONTROL) {
            mControlState = DriveControlState.HEADING_CONTROL;
        }
        mPeriodicIO.heading_setpoint = Rotation2d.fromDegrees(target_degrees);
    }

    public void setOpenLoop(ChassisSpeeds speeds) {
        mPeriodicIO.des_chassis_speeds = speeds;
        if (mControlState != DriveControlState.OPEN_LOOP) {
            mControlState = DriveControlState.OPEN_LOOP;
        }
    }

    public void setVelocity(ChassisSpeeds speeds) {
        mPeriodicIO.des_chassis_speeds = speeds;
        if (mControlState != DriveControlState.VELOCITY) {
            mControlState = DriveControlState.VELOCITY;
        }
    }

    public void setTrajectory(Trajectory trajectory, Rotation2d heading) {
        if (mMotionPlanner != null) {
            mMotionPlanner.setTrajectory(trajectory, heading, getPose());
            mControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    public void setAutoHeading(Rotation2d new_heading) {
        mMotionPlanner.setTargetHeading(new_heading);
    }

    // Stops drive without orienting modules
    public synchronized void stopModules() {
        List<Rotation2d> orientations = new ArrayList<>();
        for (ModuleState moduleState : getModuleStates()) {
            orientations.add(moduleState.angle);
        }
        orientModules(orientations);
    }

    public synchronized void orientModules(List<Rotation2d> orientations) {
        if (mControlState != DriveControlState.FORCE_ORIENT) {
            mControlState = DriveControlState.FORCE_ORIENT;
        }
        for (int i = 0; i < mModules.length; ++i) {
            mPeriodicIO.des_module_states[i] = ModuleState.fromSpeeds(orientations.get(i), 0.0);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mPeriodicIO.des_chassis_speeds = new ChassisSpeeds();
                mControlState = DriveControlState.OPEN_LOOP;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    switch (mControlState) {
                        case PATH_FOLLOWING:
                            updatePathFollower();
                            break;
                        case HEADING_CONTROL:
                            break;
                        case AUTO_BALANCE:
                        case OPEN_LOOP:
                        case VELOCITY:
                        case FORCE_ORIENT:
                            break;
                        default:
                            stop();
                            break;
                    }
                    updateSetpoint();
                    mOdometry.update(mPeriodicIO.heading, getModuleStates());
                }
            }

            @Override
            public void onStop(double timestamp) {
                mPeriodicIO.des_chassis_speeds = new ChassisSpeeds();
                mControlState = DriveControlState.OPEN_LOOP;
            }
        });
    }

    private double last_pitch = 0.0;

    @Override
    public void readPeriodicInputs() {
        for (SwerveModule swerveModule : mModules) {
            swerveModule.readPeriodicInputs();
        }

        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.meas_module_states = getModuleStates();
        mPeriodicIO.meas_chassis_speeds = kKinematics.toChassisSpeeds(mPeriodicIO.meas_module_states);
        mPeriodicIO.heading = mPigeon.getYaw();
        mPeriodicIO.pitch = mPigeon.getPitch();

        smoothed_pitch_velocity.addNumber((mPeriodicIO.pitch.getDegrees() - last_pitch) / Constants.kLooperDt);
        last_pitch = mPeriodicIO.pitch.getDegrees();
    }

    private void updatePathFollower() {
        if (mControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();
            ChassisSpeeds output = mMotionPlanner.update(getPose(), now);
            mPeriodicIO.des_chassis_speeds = output;
        }
    }

    public void setAutoSpinFast(boolean spin) {
        spinFastDuringAuto = spin;
    }

    private void updateSetpoint() {
        if (mControlState == DriveControlState.FORCE_ORIENT)
            return;

        Pose2d robot_pose_vel = new Pose2d(mPeriodicIO.des_chassis_speeds.vxMetersPerSecond * Constants.kLooperDt,
                mPeriodicIO.des_chassis_speeds.vyMetersPerSecond * Constants.kLooperDt,
                Rotation2d.fromRadians(mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond * Constants.kLooperDt));
        Twist2d twist_vel = new Pose2d().log(robot_pose_vel);
        ChassisSpeeds wanted_speeds = new ChassisSpeeds(
                twist_vel.dx / Constants.kLooperDt, twist_vel.dy / Constants.kLooperDt,
                twist_vel.dtheta / Constants.kLooperDt);

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

        ModuleState[] prev_module_states = mPeriodicIO.des_module_states.clone(); // Get last setpoint to get
                                                                                  // differentials
        ChassisSpeeds prev_chassis_speeds = kKinematics.toChassisSpeeds(prev_module_states);
        ModuleState[] target_module_states = kKinematics.toModuleStates(wanted_speeds);

        if (wanted_speeds.epsilonEquals(new ChassisSpeeds(), Util.kEpsilon)) {
            for (int i = 0; i < target_module_states.length; i++) {
                target_module_states[i].speedMetersPerSecond = 0.0;
                target_module_states[i].angle = prev_module_states[i].angle;
            }
        }

        double dx = wanted_speeds.vxMetersPerSecond - prev_chassis_speeds.vxMetersPerSecond;
        double dy = wanted_speeds.vyMetersPerSecond - prev_chassis_speeds.vyMetersPerSecond;
        double domega = wanted_speeds.omegaRadiansPerSecond - prev_chassis_speeds.omegaRadiansPerSecond;

        double max_velocity_step = mKinematicLimits.kMaxAccel * Constants.kLooperDt;
        double min_translational_scalar = 1.0;

        if (max_velocity_step < Double.MAX_VALUE * Constants.kLooperDt) {
            // Check X
            double x_norm = Math.abs(dx / max_velocity_step);
            min_translational_scalar = Math.min(min_translational_scalar, x_norm);

            // Check Y
            double y_norm = Math.abs(dy / max_velocity_step);
            min_translational_scalar = Math.min(min_translational_scalar, y_norm);

            min_translational_scalar *= max_velocity_step;
        }

        double max_omega_step = mKinematicLimits.kMaxAngularAccel * Constants.kLooperDt;
        double min_omega_scalar = 1.0;

        if (max_omega_step < Double.MAX_VALUE * Constants.kLooperDt) {
            double omega_norm = Math.abs(domega / max_omega_step);
            min_omega_scalar = Math.min(min_omega_scalar, omega_norm);

            min_omega_scalar *= max_omega_step;
        }

        SmartDashboard.putNumber("Accel", min_translational_scalar);

        if (!spinFastDuringAuto) {
            wanted_speeds = new ChassisSpeeds(
                    prev_chassis_speeds.vxMetersPerSecond + dx * min_translational_scalar,
                    prev_chassis_speeds.vyMetersPerSecond + dy * min_translational_scalar,
                    prev_chassis_speeds.omegaRadiansPerSecond + domega * min_omega_scalar);
        } else {
            wanted_speeds = new ChassisSpeeds(
                    prev_chassis_speeds.vxMetersPerSecond + dx * min_translational_scalar,
                    prev_chassis_speeds.vyMetersPerSecond + dy * min_translational_scalar,
                    -12);
        }

        ModuleState[] real_module_setpoints = kKinematics.toModuleStates(wanted_speeds);
        mPeriodicIO.des_module_states = real_module_setpoints;

    }

    public void resetModulesToAbsolute() {
        for (SwerveModule module : mModules) {
            module.resetToAbsolute();
        }
    }

    public void zeroGyro() {
        zeroGyro(0.0);
    }

    public void zeroGyro(double reset) {
        mPigeon.setYaw(reset);
        mPigeon.setPitch(0.0);
    }

    public void setNeutralBrake(boolean brake) {
        for (SwerveModule swerveModule : mModules) {
            swerveModule.setDriveNeutralBrake(brake);
        }
    }

    @Override
    public void writePeriodicOutputs() {
        for (int i = 0; i < mModules.length; i++) {
            if (mControlState == DriveControlState.OPEN_LOOP || mControlState == DriveControlState.HEADING_CONTROL) {
                mModules[i].setDesiredState(mPeriodicIO.des_module_states[i], true);
            } else if (mControlState == DriveControlState.PATH_FOLLOWING || mControlState == DriveControlState.VELOCITY
                    ||
                    mControlState == DriveControlState.AUTO_BALANCE
                    || mControlState == DriveControlState.FORCE_ORIENT) {
                mModules[i].setDesiredState(mPeriodicIO.des_module_states[i], false);
            }
        }

        for (SwerveModule swerveModule : mModules) {
            swerveModule.writePeriodicOutputs();
        }
    }

    public ModuleState[] getModuleStates() {
        ModuleState[] states = new ModuleState[4];
        for (SwerveModule mod : mModules) {
            states[mod.moduleNumber()] = mod.getState();
        }
        return states;
    }

    public Pose2d getPose() {
        return mOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometryReset = true;
        Pose2d wanted_pose = pose;
        Rotation2d wantedRotationReset = pose.getRotation();
        // may need to bring back later:
        // if (Robot.flip_trajectories)  {
        //     wantedRotationReset.rotateBy(Rotation2d.fromDegrees(180));
        // }
        mOdometry.resetPosition(getModuleStates(), wanted_pose);
        zeroGyro(wantedRotationReset.getDegrees());
    }

    public boolean readyForAuto() {
        return odometryReset;
    }

    public boolean isDoneWithTrajectory() {
        if (mControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isFinished();
    }

    public Rotation2d getHeading() {
        return mPigeon.getYaw();
    }

    public DriveMotionPlanner getTrajectoryFollower() {
        return mMotionPlanner;
    }

    public SwerveConstants.KinematicLimits getKinematicLimits() {
        return mKinematicLimits;
    }

    public static class PeriodicIO {
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

        // Outputs
        ModuleState[] des_module_states = new ModuleState[] {
                new ModuleState(),
                new ModuleState(),
                new ModuleState(),
                new ModuleState()
        };
        Pose2d path_setpoint = new Pose2d();
        Rotation2d heading_setpoint = new Rotation2d();
    }

    @Override
    public void outputTelemetry() {
        if (Constants.disableExtraTelemetry) {
            return;
        }
        for (SwerveModule module : mModules) {
            module.outputTelemetry();
        }
        SmartDashboard.putNumber("Pitch", mPeriodicIO.pitch.getDegrees());
        // SmartDashboard.putNumber("Delta Pitch",
        // smoothed_pitch_velocity.getAverage());
        SmartDashboard.putString("drive control state", mControlState.toString());
        SmartDashboard.putNumber("Drive X Velocity", getMeasuredXVelocity());
        SmartDashboard.putNumber("ROBOT HEADING", getHeading().getDegrees());
    }

    @Log
    public double getTimestamp() {
        return mPeriodicIO.timestamp;
    }

    @Log
    public double getPitch() {
        return mPigeon.getPitch().getDegrees();
    }

    @Log
    public double getSmoothedPitchVelocity() {
        return smoothed_pitch_velocity.getAverage();
    }

    @Log
    public double getXTrajectoryError() {
        return mMotionPlanner.getXError(getPose().getX(), Timer.getFPGATimestamp());
    }

    @Log
    public double getYTrajectoryError() {
        return mMotionPlanner.getYError(getPose().getY(), Timer.getFPGATimestamp());
    }

    @Log
    public double getRotationError() {
        return mMotionPlanner.getRotationalError(getPose().getRotation().getDegrees());
    }

    @Log
    public double getTrajectoryX() {
        return mMotionPlanner.getXError(0.0, Timer.getFPGATimestamp());
    }

    @Log
    public double getTrajectoryY() {
        return mMotionPlanner.getYError(0.0, Timer.getFPGATimestamp());
    }

    @Log
    public double getTrajectoryHeading() {
        return mMotionPlanner.getRotationalTarget();
    }

    @Log
    public double getMeasuredXVelocity() {
        return mPeriodicIO.meas_chassis_speeds.vxMetersPerSecond;
    }

    @Log
    public double getMeasuredYVelocity() {
        return mPeriodicIO.meas_chassis_speeds.vyMetersPerSecond;
    }

    @Log
    public double getMeasuredOmega() {
        return mPeriodicIO.meas_chassis_speeds.omegaRadiansPerSecond;
    }

    @Log
    public double getPoseX() {
        return getPose().getX();
    }

    @Log
    public double getPoseY() {
        return getPose().getY();

    }

    @Log
    public double getThetaDegrees() {
        return getPose().getRotation().getDegrees();

    }

    @Log
    public double getTargetXSpeed() {
        return mPeriodicIO.des_chassis_speeds.vxMetersPerSecond;
    }

    @Log
    public double getTargetYSpeed() {
        return mPeriodicIO.des_chassis_speeds.vyMetersPerSecond;
    }

    @Log
    public double getTargetOmega() {
        return mPeriodicIO.des_chassis_speeds.omegaRadiansPerSecond;
    }

    @Log
    public String getControlState() {
        return mControlState.toString();
    }

    @Log
    public int getBalanceStep() {
        return balance_step;
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return false;
    }

    // Auto engage controls

    double flap_trigger_angle = -5.0; // Less than
    double platform_trigger_angle = -12.0; // Equal to
    double balance_trigger_velocity = 8.0; // Greater than
    int balance_step = 0;

    private MovingAverage smoothed_pitch_velocity = new MovingAverage(10);
}