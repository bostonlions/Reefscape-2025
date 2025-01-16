// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.lib.Util.Conversions;
import frc.robot.lib.swerve.SwerveModule.SwerveModuleConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public class Constants {
    // toggle constants between comp bot and practice bot ("epsilon")
    public static boolean isBeta = false;
    public static boolean isComp = true;

    public static boolean isCompBot() {
        return isComp;
    }

    public static boolean isBetaBot() {
        return isBeta;
    }

    // Disables extra smart dashboard outputs that slow down the robot
    public static final boolean disableExtraTelemetry = true;

    public static final boolean isManualControlMode = false;

    // robot loop time
    public static final double kLooperDt = 0.02;

    /* Control Board */
    public static final double kTriggerThreshold = 0.2;

    public static final double stickDeadband = 0.05;
    public static final int leftXAxis = 0;
    public static final int leftYAxis = 1;
    public static final int rightXAxis = 3;
    public static final int rightYAxis = 4;

    public static final class SwerveConstants {
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.25);
        public static final double wheelBase = Units.inchesToMeters(20.25);

        public static final double wheelDiameter = Units.inchesToMeters(3.85);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        // this is for the comp bot
        // public static final double driveGearRatio = 6.75; //flipped gear ratio
        // https://docs.wcproducts.com/wcp-swervex/general-info/ratio-options
        // public static final double angleGearRatio = 15.43; //8:32:24--14:72 = 15.43
        // ratio

        // can tune this value by driving a certain distance and multiplying a const to
        // fix the error
        public static final double driveGearRatio = ((5.3 / 1.07) / 1.04); // it's 4.76:1
        public static final double angleGearRatio = 21.4285714;// (150/7);// 10.29; // 72:14:24:12

        public static final Translation2d[] swerveModuleLocations = {
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0)
        };

        /* Swerve Current Limiting - very neccesary! */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 62;
        public static final int drivePeakCurrentLimit = 65;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.32 / 12);
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.8; // meters per second MAX : 5.02 m/s
        public static final double maxAngularVelocity = 8.0;

        // Max out at 85% to make sure speeds are attainable (4.6 mps)
        public static final double maxAttainableSpeed = maxSpeed * 0.85;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6; // 0.3 falcon - higher with kraken
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 4.0;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 12.0 / Conversions.MPSToRPS(maxSpeed, wheelCircumference, driveGearRatio);

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Motor Inverts */
        public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive; // false

        /* Controller Invert */
        public static final boolean invertYAxis = false;
        public static final boolean invertRAxis = false;
        public static final boolean invertXAxis = true;

        public static class KinematicLimits {
            public double kMaxDriveVelocity = maxSpeed; // m/s
            public double kMaxAccel = Double.MAX_VALUE; // m/s^2
            public double kMaxAngularVelocity = maxAngularVelocity; // rad/s
            public double kMaxAngularAccel = Double.MAX_VALUE; // rad/s^2

            public KinematicLimits() {}
            public KinematicLimits(
                double kMaxDriveVelocity, double kMaxAccel, double kMaxAngularVelocity, double kMaxAngularAccel
            ) {
                this.kMaxDriveVelocity = kMaxDriveVelocity;
                this.kMaxAccel = kMaxAccel;
                this.kMaxAngularVelocity = kMaxAngularVelocity;
                this.kMaxAngularAccel = kMaxAngularAccel;
            }
        }

        public static final KinematicLimits kUncappedLimits = new KinematicLimits();

        public static final KinematicLimits kScoringLimits = new KinematicLimits(
            2.0, Double.MAX_VALUE, Math.PI, 10 * Math.PI
        );

        public static final KinematicLimits kLoadingStationLimits = new KinematicLimits(
            1.5, Double.MAX_VALUE, maxAngularVelocity, Double.MAX_VALUE
        );

        public static final KinematicLimits kAutoLimits = new KinematicLimits(
            maxAttainableSpeed, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
        );

        /***
         * MODULE SPECIFIC CONSTANTS **
         *
         * which way to zero modules?
         * if you tip the robot up on its back side, align the bevel gears to the right
         * side (from lookers perspective) on all the wheels. Make sure all the wheels are
         * in line, then record canCoder offset values in shuffleboard
         *
         * Zero them so that odometry is x positive going forwards and y positive going left
         */

        /*** MODULE SPECIFIC CONSTANTS ***/
        /* Front Left Module - Module 0 */
        private static final double mod0BetaAngleOffset = 39.63;
        private static final double mod0CompAngleOffset = 39.63;
        public static final SwerveModuleConstants Mod0 = new SwerveModuleConstants(
            Ports.FL_DRIVE, Ports.FL_ROTATION, Ports.FL_CANCODER,
            isComp ? mod0CompAngleOffset : mod0BetaAngleOffset
        );

        /* Front Right Module - Module 1 */
        private static final double mod1BetaAngleOffset = 169.62;
        private static final double mod1CompAngleOffset = 169.62;
        public static final SwerveModuleConstants Mod1 = new SwerveModuleConstants(
            Ports.FR_DRIVE, Ports.FR_ROTATION, Ports.FR_CANCODER,
            isComp ? mod1CompAngleOffset : mod1BetaAngleOffset
        );

        /* Back Left Module - Module 2 */
        private static final double mod2BetaAngleOffset = 181.58;
        private static final double mod2CompAngleOffset = 181.58;
        public static final SwerveModuleConstants Mod2 = new SwerveModuleConstants(
            Ports.BL_DRIVE, Ports.BL_ROTATION, Ports.BL_CANCODER,
            isComp ? mod2CompAngleOffset : mod2BetaAngleOffset
        );

        /* Back Right Module - Module 3 */
        private static final double mod3BetaAngleOffset = 278.52;
        private static final double mod3CompAngleOffset = 278.52;
        public static final SwerveModuleConstants Mod3 = new SwerveModuleConstants(
            Ports.BR_DRIVE, Ports.BR_ROTATION, Ports.BR_CANCODER,
            isComp ? mod3CompAngleOffset : mod3BetaAngleOffset
        );

        public static TalonFXConfiguration swerveDriveFXConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.SupplyCurrentLimitEnable = driveEnableCurrentLimit;
            config.CurrentLimits.SupplyCurrentLowerLimit = driveContinuousCurrentLimit;
            config.CurrentLimits.SupplyCurrentLimit = drivePeakCurrentLimit;
            config.CurrentLimits.SupplyCurrentLowerTime = drivePeakCurrentDuration;

            config.Voltage.PeakForwardVoltage = 12.0;
            config.Voltage.PeakReverseVoltage = -12.0;

            config.Slot0.kP = driveKP;
            config.Slot0.kI = driveKI;
            config.Slot0.kD = driveKD;

            config.MotorOutput.NeutralMode = driveNeutralMode;
            config.MotorOutput.Inverted = driveMotorInvert;

            config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
            config.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;
            return config;
        }

        public static TalonFXConfiguration swerveAngleFXConfig() {
            TalonFXConfiguration angleConfig = new TalonFXConfiguration();
            angleConfig.CurrentLimits.SupplyCurrentLimitEnable = angleEnableCurrentLimit;
            angleConfig.CurrentLimits.SupplyCurrentLowerLimit = angleContinuousCurrentLimit;
            angleConfig.CurrentLimits.SupplyCurrentLimit = anglePeakCurrentLimit;
            angleConfig.CurrentLimits.SupplyCurrentLowerTime = anglePeakCurrentDuration;

            angleConfig.Slot0.kP = angleKP;
            angleConfig.Slot0.kI = angleKI;
            angleConfig.Slot0.kD = angleKD;
            angleConfig.Slot0.kV = angleKF;

            angleConfig.MotorOutput.NeutralMode = angleNeutralMode;
            angleConfig.MotorOutput.Inverted = angleMotorInvert;

            return angleConfig;
        }

        public static CANcoderConfiguration swerveCancoderConfig() {
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
            config.MagnetSensor.SensorDirection = canCoderInvert;
            return config;
        }
    }

    public static final class SnapConstants {
        public static final double kP = 6.0;
        public static final double kI = 0.5;
        public static final double kD = 0.2;
        public static final double snapTimeout = 0.25;
        public static final double snapEpsilon = 1.0;
    }

    public static final class AutoConstants {
        public static final double kPXController = 6.7;
        public static final double kPYController = 6.7;

        public static final double kDXController = 0.0;
        public static final double kDYController = 0.0;

        public static final double kPThetaController = 2.75; // was 2, changed to 4 -- faster it turns = more wheels slip

        // Constraint for the motion profilied robot angle controller (Radians)
        public static final double kMaxAngularSpeed = 2.0 * Math.PI;
        public static final double kMaxAngularAccel = 2.0 * Math.PI * kMaxAngularSpeed;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeed, kMaxAngularAccel);

        // Static factory for creating trajectory configs
        public static TrajectoryConfig createConfig(double maxSpeed, double maxAccel, double startSpeed, double endSpeed) {
            TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAccel);
            config.setStartVelocity(startSpeed);
            config.setEndVelocity(endSpeed);
            config.addConstraint(new CentripetalAccelerationConstraint(10.0));
            return config;
        }
    }

    public static final class VisionAlignConstants {
        public static final double kP = 6.37;
        public static final double kI = 0.0;
        public static final double kD = 0.10;
        public static final double kTimeout = 0.25;
        public static final double kEpsilon = 5.0;

        // Constraints for the profiled angle controller
        public static final double kMaxAngularSpeedRadiansPerSecond = 2.0 * Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = 10.0 * Math.PI;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        /* April Tag Chase */

        public static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 1);
        public static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(1.5, 1);
        public static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

        public static final int TAG_TO_CHASE = 1;
        public static final Transform3d TAG_TO_GOAL = new Transform3d(
                new Translation3d(0.0, 0.0, 0.0),
                new Rotation3d(0.0, 0.0, Math.PI));

        public static final TrajectoryConfig TAG_TRAJECTORY_CONFIG = Constants.AutoConstants.createConfig(
            2.5, 2.0, 0.0, 0.0
        );

        public static final double POSITION_OFF = 0.1;

        /**
         * Standard deviations of model states. Increase these numbers to trust your
         * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ,
         * with units in meters and radians, then meters.
         */
        public static final Vector<N3> STATE_STDS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

        /**
         * Standard deviations of the vision measurements. Increase these numbers to
         * trust global measurements from vision less. This matrix is in the form
         * [x, y, theta]ᵀ, with units in meters and radians.
         */
        public static final Vector<N3> VISION_STDS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));

    }

    public static final class MacAddressConstants {
        public static final byte[] COMP_ADDRESS = new byte[] {
                // values are for comp -> 00:80:2f:35:b8:ca
                (byte) 0x00, (byte) 0x80, (byte) 0x2f, (byte) 0x35, (byte) 0xb8, (byte) 0xca
        };

        public static final byte[] BETA_ADDRESS = new byte[] {
                // values are for beta -> 00:80:2f:34:0B:9B
                (byte) 0x00, (byte) 0x80, (byte) 0x2f, (byte) 0x34, (byte) 0x0b, (byte) 0x9b
        };
    }

    /*** SUBSYSTEM CONSTANTS ***/

    public static final class PivotConstants {
        public static final double kStatorCurrentLimit = 80.0;
        public static final double CANCODER_OFFSET = 106.1; // -4.8 so it never gets to -360 and breaks now it's 4.8 on
                                                            // 3/27
        public static final double kPositionError = 2; // 2 degrees of error

        public static final double gravityFeedforward = 0.0; // idk how this works

        public static final double PivotGearRatio = (25) * (74 / 18);// 25:1 74:18 revolutions of the pivot per 1
                                                                     // rotation of the motor

        public static final int kMinAngle = 5; // deg
        public static final int kMaxAngle = 95; // deg

        /* State Positions */
        public static final double kSourceIntakeAngle = 68;
        public static final double kSourceLoadShooterAngle = 41; // if anything, lower
        public static final double kStowAngle = 4.8;
        public static final double kAmpScoreAngle = 88; // was 88

        // SHOOTING ANGLES
        public static final double kShootAgainstSubwooferAngle = 58.5;
        public static final double kShootAgainstPodiumAngle = 35;
        public static final double kPassNoteFromMidAngle = 56;

        // Autos
        public static final double kStage2PieceAngle = 44.5; // 46 at dcmp?
        public static final double kMid2PieceAngle = 55; // 53 - 1;
        public static final double kAmp2PieceAngle = 39;

        public static final double kShootLoadAngle = 56; // changed from 54

        /* CLIMB CONSTANTS */
        public static final double kClimbInitAngle1 = 63; // deg
        public static final double kClimbInitAngle2 = 68; // deg
        public static final double kPullOntoChainAngle1 = 20;
        public static final double kPullOntoChainAngle2 = 6.75; // once elevator is down, goto this angle
        public static final double kExtendOffChainAngle1 = 16.8; // Once chain hooked go up to this angle and wait for
                                                                 // release
        public static final double kExtendOffChainAngle2 = 32; // Angle before abrupt flip over to trap
        public static final double kExtendToScoreTrapAngle1 = 67; // angle when pressed up against trap wall
        public static final double kExtendToScoreTrapAngle2 = 97;// 97; // angle when pressed up against trap wall

        /* CLIMB DOWN CONSTANTS */
        public static final double kDeclimbAngle1 = 50.7;
        public static final double kDeclimbAngle2 = 31.7;
        public static final double kDeclimbAngle3 = 7.5; // was 8.2
        public static final double kDeclimbAngle4 = 78;

        public static final double kIntakeCruiseVelocity = 40;
        public static final double kIntakeAcceleration = 80;

        public static CANcoderConfiguration pivotCancoderConfig() {
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
            return config;
        }

        public static TalonFXConfiguration pivotFastMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            // TODO: do any of these configs even matter?
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLowerLimit = 25; // start off pretty low when initially testing
            config.CurrentLimits.SupplyCurrentLimit = 40;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            config.Slot0.kP = 0.7;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 150;
            config.MotionMagic.MotionMagicExpo_kA = 0.7;
            config.MotionMagic.MotionMagicAcceleration = 120;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            return config;
        }

        public static TalonFXConfiguration pivotSlowMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            // TODO: do any of these configs even matter?
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLowerLimit = 30; // start off pretty low
            config.CurrentLimits.SupplyCurrentLimit = 40;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            config.Slot0.kP = 0.7;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 35;
            config.MotionMagic.MotionMagicExpo_kA = 0.7;
            config.MotionMagic.MotionMagicAcceleration = 80;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            return config;
        }

        public static TalonFXConfiguration pivotCurlMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            // TODO: do any of these configs even matter?
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLowerLimit = 30; // start off pretty low
            config.CurrentLimits.SupplyCurrentLimit = 40;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            config.Slot0.kP = 0.7;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 15;
            config.MotionMagic.MotionMagicExpo_kA = 0.7;
            config.MotionMagic.MotionMagicAcceleration = 30;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            return config;
        }

    }

    public static final class WristConstants {
        public static final double CANCODER_OFFSET = 117.36;// +3.3 so it never gets to 0

        public static final double kGearRatio = 25; // 25:1

        public static final double kMinPosition = 0; // degrees
        public static final double kMaxPosition = 200; // degrees

        public static final double kSourceIntakeAngle = 294;
        public static final double kStowAngle = 155;
        public static final double kAmpScoreAngle = 164; // was 169
        public static final double kloadShooterAngle = 118;// 118.8;

        public static final double kShootAngle = 118; // cancoder should rest at 118.8, so that when shooting the wrist
                                                      // is pulled down against elevator

        public static final double kClimbAngle1 = 140;
        public static final double kClimbFirstPressAngle = 192; // lowered from 200
        public static final double kClimbSecondPressAngle = 218; // lowered from 225
        // public static final double kClimbAngle3 = 165;
        public static final double kClimbScoreInTrapAngle = 160; // ~200?

        public static final double kIntakeCruiseVelocity = 50;
        public static final double kIntakeAcceleration = 120;

        public static TalonFXConfiguration wristMotorClimbConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLowerLimit = 15; // start off pretty low
            config.CurrentLimits.SupplyCurrentLimit = 20;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            config.Slot0.kP = 0.6;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 80;
            config.MotionMagic.MotionMagicExpo_kA = 0.2;
            config.MotionMagic.MotionMagicAcceleration = 120;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            // down to intake is increasing, up to load is decreasing
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            return config;
        }

        public static TalonFXConfiguration wristMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLowerLimit = 15; // start off pretty low
            config.CurrentLimits.SupplyCurrentLimit = 20;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            config.Slot0.kP = 0.6;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 100;
            config.MotionMagic.MotionMagicExpo_kA = 0.2;
            config.MotionMagic.MotionMagicAcceleration = 170;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            // down to intake is increasing, up to load is decreasing
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            return config;
        }

        public static CANcoderConfiguration wristCancoderConfig() {
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
            config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
            return config;
        }
    }

    public static final class ElevatorConstants {
        public static final int kMaxVoltage = 12;
        public static final double kGearRatio = 12;
        public static final double kWheelCircumference = Conversions.inchesToMeters(1.625) * Math.PI;
        public static final double kPositionError = Conversions.inchesToMeters(0.5);

        public static final double kMinHeight = 0; // meters
        public static final double kMaxHeight = 0.7;

        public static final double kStowHeight = 0.018;
        public static final double kAmpScoreHeight = 0.22 + Conversions.inchesToMeters(1);

        /* SHOOTING */
        public static final double kloadShooterInitialHeight = 0.32 + Conversions.inchesToMeters(1.7);
        public static final double kloadShooterFinalHeight = 0.034 + Conversions.inchesToMeters(5.5);
        public static final double kShootHeight = 0.26;

        /* INTAKING */
        public static final double kIntakeCruiseVelocity = 90;
        public static final double kIntakeAcceleration = 120;

        public static final double kFloorIntakeHeight = 0.29;
        public static final double kSourceIntakeHeight = 0.064;
        public static final double kSourceLoadShooterHeight = 0.22;

        /* CLIMB */
        public static final double kClimbInitHeight = 0.32 + Conversions.inchesToMeters(4.75); // initial height going
                                                                                               // up
        public static final double kMaxClimbInitHeight = 0.32 + Conversions.inchesToMeters(8); // TODO: set this
        // to chain
        public static final double kPullOntoChainHeight = Conversions.inchesToMeters(0.25); // height of the elevator
                                                                                            // when transfering chain

        public static final double kExtendOffChain1 = 0.054;
        public static final double kExtendOffChain2 = 0.126;
        public static final double kExtendOffChain3 = 0.26 - Conversions.inchesToMeters(1); // to go within height
                                                                                            // limits
        public static final double kExtendToScoreTrapHeight = 0.447 - Conversions.inchesToMeters(3.5); // height of the
                                                                                                       // elvator when
                                                                                                       // scoring in the
                                                                                                       // trap

        /* De Climb */
        public static final double kDeclimbHeight1 = 0.267;
        public static final double kDeclimbHeight2 = 0.06;
        public static final double kDeclimbHeight3 = 0.02;
        public static final double kDeclimbHeight4 = 0.32 + Conversions.inchesToMeters(2);

        public static TalonFXConfiguration elevatorFastMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLowerLimit = 30; // start off pretty low
            config.CurrentLimits.SupplyCurrentLimit = 20;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            config.Slot0.kP = 0.6;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 140;
            config.MotionMagic.MotionMagicExpo_kA = 0.3;
            config.MotionMagic.MotionMagicAcceleration = 300;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            return config;
        }

        public static TalonFXConfiguration elevatorSlowMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            // TODO: do any of these configs even matter?
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLowerLimit = 30; // start off pretty low
            config.CurrentLimits.SupplyCurrentLimit = 20;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            config.Slot0.kP = 0.6;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 140; // was 50 for 1st comp
            config.MotionMagic.MotionMagicExpo_kA = 0.2;
            config.MotionMagic.MotionMagicAcceleration = 140;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            return config;
        }

        public static TalonFXConfiguration elevatorCurlMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            // TODO: do any of these configs even matter?
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLowerLimit = 35; // start off pretty low
            config.CurrentLimits.SupplyCurrentLimit = 30;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            config.Slot0.kP = 0.6;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 140; // TODO: change this
            config.MotionMagic.MotionMagicExpo_kA = 0.2;
            config.MotionMagic.MotionMagicAcceleration = 200;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            return config;
        }

        public static final double[][] groundIntakeWristPositionsOut = {
                // @0 --> position of elevator (in meters)
                // @1 --> position of wrist (in degrees)
                // @2 --> position of the pivot(in degrees)
                { 0.004, 335, 5 },
                { 0.03, 337, 6 },
                { 0.052, 340, 8 },
                { 0.075, 341, 8 },
                { 0.1, 343, 8 },
                { 0.125, 345, 8 },
                { 0.15, 348, 7 },
                { 0.175, 354, 7 },
                { 0.2, 356, 7 },
                { 0.215, 358, 7 },
                { 0.23, 359.7 + 3, 7 },
                { 0.235, 359.7, 7 },
                { 0.237, 359, 5.5 } // really 0.275, but less so that everything else goes into position
        };

        public static final double[][] groundIntakeWristPositionsIn = {
                // @0 --> position of elevator (in meters)
                // @1 --> position of wrist (in degrees)
                // @2 --> position of the pivot(in degrees)
                { 0.004, 190, 4.8 },
                { 0.03, 200, 4.8 },
                { 0.052, 210, 5 },
                { 0.075, 215, 5 },
                { 0.1, 235, 6 },
                { 0.125, 290, 7 },
                { 0.15, 300, 8 },
                { 0.175, 328, 9 },
                { 0.2, 330, 10 },
                { 0.215, 333, 11 },
                { 0.23, 335, 11 },
                { 0.235, 340, 11 },
                { 0.24, 345, 10 } // really 0.275, but less so that everything else goes into position
        };
    }

    public static final class EndEffectorConstants {
        // SHOOTING RPM's
        public static final double kSubwooferRPM = 5050; // 5000 //tunes to 5080
        public static final double kShootFastRPM = 6550; // tunes to 6560 rpm for passing and shooting from furthur away
        public static final double kPassRPM = 6300;
        // INTAKE/OUTTAKE DEMANDS
        public static final double kSourceIntakeDemand = 0.35; // was 0.35 with more resistance end effector
        public static final double kGroundIntakeDemand = 0.475; // was 0.58 for more resistance eff
        public static final double kOuttakingDemandTop = -0.50;
        public static final double kOuttakingDemandBottom = -0.55;

        // PID TUNING
        // SUBWOOFER
        public static final double kFFTopSubwoofer = 0.000153; // tunes the subwoofer shot
        public static final double kFFBottomSubwoofer = 0.000154; // tunes the subwoofer shot
        // FAST
        public static final double kFFTopFast = 0.000154; // tunes the note passing
        public static final double kFFBottomFast = 0.000155; // tunes the note passing

        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;

        public static final double kPSubWof = 0.00022;
        public static final double kPFast = 0.00022;
    }

    public static final class ShooterConstants {
        public static final double kLoadShooterDemand = -0.65;
        public static final double kSlingshotDemand = 0.95;

        public static TalonFXConfiguration shooterMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLowerLimit = 30; // start off pretty low
            config.CurrentLimits.SupplyCurrentLimit = 20;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            config.MotionMagic.MotionMagicExpo_kA = 0.2;
            config.MotionMagic.MotionMagicAcceleration = 300;

            config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

            return config;
        }
    }

    public static final class ClimberHookConstants {
        public static final double kHookAngle = 88; // degrees
        public static final double kDeclimb1Angle = 88;
        public static final double kUnhookAngle = 0; // so we don't have to worry about resetting it while practicing
        public static final double kMaxAngle = 131;
        public static final double kMinAngle = 0;
        public static final double kGearRatio = 45;

        public static TalonFXConfiguration climberHookMotorConfig() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.CurrentLimits.SupplyCurrentLimitEnable = false;
            config.CurrentLimits.SupplyCurrentLowerLimit = 10; // start off pretty low
            config.CurrentLimits.SupplyCurrentLimit = 20;
            config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

            config.Slot0.kP = 0.6;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0;
            config.Slot0.kV = 0.0;

            config.MotionMagic.MotionMagicCruiseVelocity = 160;
            config.MotionMagic.MotionMagicExpo_kA = 0.3;
            config.MotionMagic.MotionMagicAcceleration = 400;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            return config;
        }
    }

    public static final class ControllerConstants {
        public static final boolean isMamboController = true; // this overrides everything
        public static final boolean isControllerOne = true;

        // Controller 1 left side:
        public static final double ControllerOneLeftThrottleZero = -0.125;
        public static final double ControllerOneLeftYawZero = 0.039370;

        public static final double ControllerOneLeftThrottleHigh = 0.787402;
        public static final double ControllerOneLeftThrottleLow = 0.968750;

        public static final double ControllerOneLeftYawHigh = 0.86612;
        public static final double ControllerOneLeftYawLow = 0.77338;

        // Controller 1 right side:
        public static final double ControllerOneRightThrottleZero = 0.055118;
        public static final double ControllerOneRightYawZero = 0.055118;

        public static final double ControllerOneRightYawHigh = 0.866142;
        public static final double ControllerOneRightYawLow = 0.765625;

        public static final double ControllerOneRightThrottleHigh = 0.732283;
        public static final double ControllerOneRightThrottleLow = 0.601563;

        // Controller 2 left side:
        public static final double ControllerTwoLeftThrottleZero = -0.023438;
        public static final double ControllerTwoLeftYawZero = -0.078125;

        public static final double ControllerTwoLeftThrottleHigh = 0.834646;
        public static final double ControllerTwoLeftThrottleLow = 0.867188;

        public static final double ControllerTwoLeftYawHigh = 0.748031;
        public static final double ControllerTwoLeftYawLow = 0.890625;

        // Controller 2 right side:
        public static final double ControllerTwoRightThrottleZero = -0.054688; // high 0.007874
        public static final double ControllerTwoRightYawZero = 0.062992;

        public static final double ControllerTwoRightYawHigh = 0.866142;
        public static final double ControllerTwoRightYawLow = 0.664063;

        public static final double ControllerTwoRightThrottleHigh = 0.669291;
        public static final double ControllerTwoRightThrottleLow = 0.664063;

        // Controller left side:
        public static final double ControllerLeftThrottleZero = isControllerOne ? ControllerOneLeftThrottleZero
                : ControllerTwoLeftThrottleZero;
        public static final double ControllerLeftYawZero = isControllerOne ? ControllerOneLeftYawZero
                : ControllerTwoLeftYawZero;

        public static final double ControllerLeftThrottleHigh = isControllerOne ? ControllerOneLeftThrottleHigh
                : ControllerTwoLeftThrottleHigh;
        public static final double ControllerLeftThrottleLow = isControllerOne ? ControllerOneLeftThrottleLow
                : ControllerTwoLeftThrottleLow;

        public static final double ControllerLeftYawHigh = isControllerOne ? ControllerOneLeftYawHigh
                : ControllerTwoLeftYawHigh;
        public static final double ControllerLeftYawLow = isControllerOne ? ControllerOneLeftYawLow
                : ControllerTwoLeftYawLow;

        // Controller right side:
        public static final double ControllerRightThrottleZero = isControllerOne ? ControllerOneRightThrottleZero
                : ControllerTwoRightThrottleZero;
        public static final double ControllerRightYawZero = isControllerOne ? ControllerOneRightYawZero
                : ControllerTwoRightYawZero;

        public static final double ControllerRightYawHigh = isControllerOne ? ControllerOneRightYawHigh
                : ControllerTwoRightYawHigh;
        public static final double ControllerRightYawLow = isControllerOne ? ControllerOneRightYawLow
                : ControllerTwoRightYawLow;

        public static final double ControllerRightThrottleHigh = isControllerOne ? ControllerOneRightThrottleHigh
                : ControllerTwoRightThrottleHigh;
        public static final double ControllerRightThrottleLow = isControllerOne ? ControllerOneRightThrottleLow
                : ControllerTwoRightThrottleLow;
    }

    // Timeout constants
    public static final int kLongCANTimeoutMs = 100;
    public static final int kCANTimeoutMs = 10;
}
