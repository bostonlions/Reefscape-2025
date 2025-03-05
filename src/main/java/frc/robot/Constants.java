// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;

import static java.util.Map.entry;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import frc.robot.lib.swerve.SwerveDriveKinematics;

public final class Constants {
    public static final class SwerveConstants {
        /** Robot loop time - but only used by swerve */
        public static final double kLooperDt = 0.02;
        /** Always ensure Gyro is CCW+ CW- */
        public static final boolean invertGyro = false;
        /** The speed reduction when drive is in strafe mode; drive speed gets divided by this */
        public static final double strafeReduction = 4.;

        /** The speed reduction when drive is in precision mode; drive speed gets divided by this */
        public static final double precisionReduction = 8.;

        // Drivetrain Constants
        public static final double trackWidth = Units.inchesToMeters(24.25);
        public static final double wheelBase = Units.inchesToMeters(24.25);

        public static final double wheelDiameter = Units.inchesToMeters(4); // was 3.85 but tire is 4 w/tread

        /** Can tune this value by driving a certain distance and multiplying a const to fix the error */
        public static final double driveGearRatio = 6.3; // also tried 6.12 // ((5.3 / 1.07) / 1.04) ?  maybe 4.7628
        public static final double angleGearRatio = 150./7;

        // Swerve Profiling Values
        public static final double maxSpeed = 5.02; // was 4.8 toggled to 2.0 meters per second MAX : 5.02 m/s
        public static final double maxAccel = Double.MAX_VALUE; //10.;
        public static final double maxAngularVelocity = 8.; //was 8.0 toggled to 2.0
        public static final double maxAngularAccel = Double.MAX_VALUE; //10.;

        /** Max out at 85% to make sure speeds are attainable (4.6 mps) */
        public static final double maxAttainableSpeed = maxSpeed * 0.85;

        public static final TalonFXConfiguration driveConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(65)
                .withSupplyCurrentLowerLimit(62)
                .withSupplyCurrentLowerTime(0.1))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.)
                .withPeakReverseVoltage(-12.))
            .withSlot0(new Slot0Configs()
                .withKP(4.)
                .withKI(0.)
                .withKD(0.)
                .withKV(12. / ((maxSpeed / (wheelDiameter * Math.PI)) * driveGearRatio)))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive))
            .withOpenLoopRamps(new OpenLoopRampsConfigs()
                .withDutyCycleOpenLoopRampPeriod(0.25)
                .withVoltageOpenLoopRampPeriod(0.25));

        public static final TalonFXConfiguration angleConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLowerLimit(25)
                .withSupplyCurrentLowerTime(0.1))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.)
                .withPeakReverseVoltage(-12.0))
            .withSlot0(new Slot0Configs()
                .withKP(0.6)
                .withKI(0.)
                .withKD(0.)
                .withKV(0.))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive));

        public static final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(1.0)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

        public static final class KinematicLimits {
            public double kMaxDriveVelocity; // m/s
            public double kMaxAccel; // m/s^2
            public double kMaxAngularVelocity; // rad/s
            public double kMaxAngularAccel; // rad/s^2

            public KinematicLimits(
                double kMaxDriveVelocity, double kMaxAccel, double kMaxAngularVelocity, double kMaxAngularAccel
            ) {
                this.kMaxDriveVelocity = kMaxDriveVelocity;
                this.kMaxAccel = kMaxAccel;
                this.kMaxAngularVelocity = kMaxAngularVelocity;
                this.kMaxAngularAccel = kMaxAngularAccel;
            }
        }

        public static final KinematicLimits kUncappedLimits = new KinematicLimits(
            maxSpeed, maxAccel, maxAngularVelocity, maxAngularAccel
        );

        // public static final KinematicLimits kScoringLimits = new KinematicLimits(
        //     2.0, Double.MAX_VALUE, Math.PI, 10 * Math.PI
        // );

        // public static final KinematicLimits kLoadingStationLimits = new KinematicLimits(
        //     1.5, Double.MAX_VALUE, maxAngularVelocity, Double.MAX_VALUE
        // );

        // public static final KinematicLimits kAutoLimits = new KinematicLimits(
        //     maxAttainableSpeed, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE
        // );

        // MODULE CANCODER ANGLE OFFSETS
        // To calibrate:
        // - Tip the robot up on its back side
        // - Align the bevel gears to the right side (from lookers perspective) on all the wheels.
        // - Make sure all the wheels are in line, then record canCoder offset values (in degrees)
        //   from shuffleboard
        public static final double FL_AngleOffset = 335.3027;
        public static final double FR_AngleOffset = 89.1211;
        public static final double BL_AngleOffset = 191.3379;
        public static final double BR_AngleOffset = 73.2129;
    }

    /** For DriveMotionPlanner */
    public static final class AutonConstants { // TODO: clean up this class and others when auton sorted out
        public static final double snapP = 6.;
        public static final double snapI = 0.5;
        public static final double snapD = 0.2;

        public static final double kPXController = 6.7;
        public static final double kPYController = 6.7;

        public static final double kDXController = 0.;
        public static final double kDYController = 0.;

        public static final double kPThetaController = 2.75; // was 2, changed to 4 -- faster it turns = more wheels slip

        // Constraints for the motion profilied robot angle controller (Radians)
        public static final double kMaxAngularSpeed = 2 * Math.PI;
        public static final double kMaxAngularAccel = 2 * Math.PI * kMaxAngularSpeed;
        public static final double kMaxCentripetalAccel = 10.;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(kMaxAngularSpeed, kMaxAngularAccel);

        public static final Translation2d[] moduleTranslations = (new SwerveDriveKinematics(SwerveConstants.wheelBase, SwerveConstants.trackWidth)).m_modules;

        public static final RobotConfig pathPlannerConfig = new RobotConfig(
            60., // TODO: get this right?
            3.873, // TODO: get this right?
            new ModuleConfig(
                SwerveConstants.wheelDiameter/2,
                SwerveConstants.maxAttainableSpeed,
                1.15, // TODO: get this right? Loooked up online and think 1.15 is correct

                // .withReduction here is VERY IMPORTANT! Autonomous drive WILL NOT WORK WITHOUT IT
                DCMotor.getKrakenX60(1).withReduction(SwerveConstants.driveGearRatio),

                111., // TODO: get this right? was 60 changed to 111 per pathplanner suggestion
                4
            ),
            moduleTranslations[0], moduleTranslations[1], moduleTranslations[2], moduleTranslations[3]
        );

        public static final PPHolonomicDriveController ppHolonomicDriveController = new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
            new PIDConstants(5., 0., 0.), // Translation PID constants
            new PIDConstants(5., 0., 0.) // Rotation PID constants
        );
    }

    public static final class ElevatorConstants {
        public static final double gearRatio = 5.;
        public static final double wheelCircumference = 0.12; // 24 teeth x 5mm belt tooth pitch - 1.625" * PI is ~0.129m
        public static final double positionError = Units.inchesToMeters(0.25);
        public static final double bottomLimitTorque = 50.;
        public static final double topLimitTorque = 70;
        public static final double limitVelocity = 0.1;
        public static final double heightTolerance = 0.005; // meters from target to consider movement complete
        public static final double resetDutyCycle = 0.1;

        // TODO: values are placeholders. Are LOAD and L1 all the same?
        /** Heights in meters */
        public enum Position { MIN, LOAD, L1, L2, L3, L4, BARGE, MAX, MANUAL }
        public static final Map<Position, Double> heights = Map.ofEntries(
            entry(Position.MIN, 0.06), // increased by 0.06
            entry(Position.LOAD, 0.063), // increased by 0.06
            entry(Position.L1, 0.18),
            entry(Position.L2, 0.36),
            entry(Position.L3, 0.75),
            entry(Position.L4, 1.402),
            entry(Position.BARGE, 1.43),
            entry(Position.MAX, 1.431),
            entry(Position.MANUAL, 0.) // not targeting a set position; controlled manually from Shuffleboard. TODO: don't need this?
        );
        /** These are the positions you can access with step up and down */
        public static final List<Position> positionOrder = List.of(
            Position.LOAD, Position.L1, Position.L2, Position.L3, Position.L4, Position.BARGE
        );

        public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLowerLimit(20)
                .withSupplyCurrentLowerTime(0.1))
            .withSlot0(new Slot0Configs()
                .withKP(.6) //.6
                .withKI(0.1)
                .withKD(0.0)
                .withKV(0.0))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(150)
                .withMotionMagicExpo_kA(0.3)
                .withMotionMagicAcceleration(150))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));
    }

    public static final class ClimberHookConstants {
        public static final double gearRatio = 80.;
        public static final double limitTorque = 100.;
        public static final double limitVelocity = 1;
        public static final double extensionTolerance = 0.1;

        // TODO: figure out these values
        public static final double extendSpeed = 2.5;
        public static final double climbSpeed = 2.5;
        public static final double nudgeSpeed = 2.1;
        public static final double climbDelay = 0.5;  // seconds to wait at DROP before climbing
        public static final double footReleaseDelay = 2.;
        public static final double footReleaseRotations = 0.6;

        public enum Position { MIN, CLIMBED, STOW, LATCH, DROP, MAX, MANUAL }
        public static final Map<Position, Double> extensions = Map.ofEntries(
            // values are in winch rotations
            // TODO figure out these values (and if the sign is even right!)
            entry(Position.MIN, -4.5),
            entry(Position.CLIMBED, -4.11),
            entry(Position.STOW, 0.), // front of prop hitting circle at front of hook
            entry(Position.LATCH, 0.30), //.99 front of prop hitting back of hook top plate
            entry(Position.DROP, 1.5), //
            entry(Position.MAX, 3.)
        );

        public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLowerLimit(30)
                .withSupplyCurrentLowerTime(0.1))
            .withSlot0(new Slot0Configs()
                .withKP(8)//.02
                .withKI(0.0)
                .withKD(0.0)
                .withKV(0.0))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(140)
                .withMotionMagicExpo_kA(0.3)
                .withMotionMagicAcceleration(Double.MAX_VALUE))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));
    }

    public static final class CoralConstants {
        public static final double gearRatio = 4.;
        public static final double loadSpeed = 0.75;
        public static final double extraLoadRotations = 0.03; // if this is 0 we never break from case statement
        public static final double unloadSpeed = 1.;
        public static final double extraUnloadRotations = 0.2;

        public static TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLowerLimit(10)
                .withSupplyCurrentLowerTime(0.1))
            .withSlot0(new Slot0Configs()
                .withKP(0.6)
                .withKI(0.0)
                .withKD(0.0)
                .withKV(0.0))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(0.5)
                .withMotionMagicExpo_kA(0.3)
                .withMotionMagicAcceleration(1.))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));
    }

    public static final class AlgaeConstants {
        public static final double angleGearRatio = (57. / 15) * 5;
        public static final double driveGearRatio = 2.;  // TODO figure out the actual number

        public static final double nudgeSpeed = 3.;

        /** Should be the amount of SECONDS it takes to stop */
        public static final double extraGroundIntakeTime = .07; //.009
        public static final double groundIntakeSpeed = 18; //was 0.5

        /** Should be the amount of SECONDS it takes to stop */
        public static final double extraReefIntakeTime = 0.4;
        public static final double reefIntakeSpeed = 20;

        /** Should be the amount of ROTATIONS it takes to stop */
        public static final double extraProcessorUnloadRotations = 1.;
        public static final double processorUnloadSpeed = 18.;

        /** Should be the amount of ROTATIONS it takes to stop */
        public static final double extraBargeUnloadRotations = 2.;
        public static final double bargeUnloadSpeed = 18;

        // Set the cancoder offset to its reading in degrees at exactly horizontal
        // to get that value, set cancoderOffset=o load the code to the robot and
        // look at smart dashboard for the CANCoder Position when it is horizontal
        public static final double cancoderOffset = -27.0;//-112.5; // -144.759;  38??

        public enum Position {
            MIN, STOW_DOWN, GROUND_INTAKE, LOADED_DOWN, PROCESSOR, STOW_UP, REEF, LOADED_UP, BARGE, MAX
        }

        // Angles in degrees from horizontal:
        public static final Map<Position, Double> angles = Map.ofEntries(
            entry(Position.MIN, -82.),
            entry(Position.STOW_DOWN, -82.),
            entry(Position.PROCESSOR, -50.),
            entry(Position.LOADED_DOWN, -63.),
            entry(Position.GROUND_INTAKE, -57.),
            entry(Position.REEF, 52.),
            entry(Position.BARGE, 95.),
            entry(Position.LOADED_UP, 95.),
            entry(Position.STOW_UP, 95.),
            entry(Position.MAX, 115.)
        );
        // // small motion range for testing only
        // public static final Map<Position, Double> angles = Map.ofEntries(
        //     entry(Position.MIN, -8.),
        //     entry(Position.STOW_DOWN, 8.),
        //     entry(Position.PROCESSOR, -5.),
        //     entry(Position.LOADED_DOWN, -6.),
        //     entry(Position.GROUND_INTAKE, -5.),
        //     entry(Position.REEF, 6.),
        //     entry(Position.BARGE, 9.),
        //     entry(Position.LOADED_UP, 9.),
        //     entry(Position.STOW_UP, 9.),
        //     entry(Position.MAX, 11.)
        // );
        public static TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20.)
                .withSupplyCurrentLowerLimit(10.)
                .withSupplyCurrentLowerTime(0.1))
            .withSlot0(new Slot0Configs()
                .withKP(0.02)
                .withKI(0.)
                .withKD(0.)
                .withKV(0.)
                .withKA(0.)
                .withKS(0.)
                .withKG(0.))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(42.)
                .withMotionMagicExpo_kA(0.03)
                .withMotionMagicAcceleration(200.))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive));

        public static TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20.)
                .withSupplyCurrentLowerLimit(10.)
                .withSupplyCurrentLowerTime(0.1))
            .withSlot0(new Slot0Configs()
                .withKP(1.65) //6.5 w 9
                .withKI(0.04) //.09 w 9
                .withKD(0.012)
                .withKV(0.0)
                .withKA(0.0)
                .withKS(0.0)
                .withKG(0.))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(8.3) //42 w9
                .withMotionMagicExpo_kA(0.3)
               // .withMotionMagicJerk(1600)
                .withMotionMagicAcceleration(1.5)) // 10 w9
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));

        public static final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                .withMagnetOffset(-cancoderOffset / 360));
    }

    public static final class ControllerConstants {
        // Max says trimming joystick input by a percent is the best way to limit speed
        // that way the autonomous system doesn't get messed up
        public static final double kInputClipping = 1; // set to 1 for 100% of joystick range

        public static final double kTriggerThreshold = 0.2;

        public static final double stickDeadband = 0.05;
        public static final int leftXAxis = 0;
        public static final int leftYAxis = 1;
        public static final int rightXAxis = 3;
        public static final int rightYAxis = 4;

        public static final boolean invertYAxis = false;
        public static final boolean invertRAxis = false;
        public static final boolean invertXAxis = true;

        // Mambo controller doesn't need any of the calibrations below
        public static final boolean isMambo = true;

        // if not Mambo there are 2 controllers with the same mechanics,
        // but different calibrations
        public static final boolean isC1 = true;

        // Controller 1 left side:
        public static final double C1LeftThrottleZero = -0.125;
        public static final double C1LeftYawZero = 0.039370;

        public static final double C1LeftThrottleHigh = 0.787402;
        public static final double C1LeftThrottleLow = 0.968750;

        public static final double C1LeftYawHigh = 0.86612;
        public static final double C1LeftYawLow = 0.77338;

        // Controller 1 right side:
        public static final double C1RightThrottleZero = 0.055118;
        public static final double C1RightYawZero = 0.055118;

        public static final double C1RightYawHigh = 0.866142;
        public static final double C1RightYawLow = 0.765625;

        public static final double C1RightThrottleHigh = 0.732283;
        public static final double C1RightThrottleLow = 0.601563;

        // Controller 2 left side:
        public static final double C2LeftThrottleZero = -0.023438;
        public static final double C2LeftYawZero = -0.078125;

        public static final double C2LeftThrottleHigh = 0.834646;
        public static final double C2LeftThrottleLow = 0.867188;

        public static final double C2LeftYawHigh = 0.748031;
        public static final double C2LeftYawLow = 0.890625;

        // Controller 2 right side:
        public static final double C2RightThrottleZero = -0.054688; // high 0.007874
        public static final double C2RightYawZero = 0.062992;

        public static final double C2RightYawHigh = 0.866142;
        public static final double C2RightYawLow = 0.664063;

        public static final double C2RightThrottleHigh = 0.669291;
        public static final double C2RightThrottleLow = 0.664063;

        // Controller left side:
        public static final double LeftThrottleZero = isC1 ? C1LeftThrottleZero : C2LeftThrottleZero;
        public static final double LeftYawZero = isC1 ? C1LeftYawZero : C2LeftYawZero;

        public static final double LeftThrottleHigh = isC1 ? C1LeftThrottleHigh : C2LeftThrottleHigh;
        public static final double LeftThrottleLow = isC1 ? C1LeftThrottleLow : C2LeftThrottleLow;

        public static final double LeftYawHigh = isC1 ? C1LeftYawHigh : C2LeftYawHigh;
        public static final double LeftYawLow = isC1 ? C1LeftYawLow : C2LeftYawLow;

        // Controller right side:
        public static final double RightThrottleZero = isC1 ? C1RightThrottleZero : C2RightThrottleZero;
        public static final double RightYawZero = isC1 ? C1RightYawZero : C2RightYawZero;

        public static final double RightYawHigh = isC1 ? C1RightYawHigh : C2RightYawHigh;
        public static final double RightYawLow = isC1 ? C1RightYawLow : C2RightYawLow;

        public static final double RightThrottleHigh = isC1 ? C1RightThrottleHigh : C2RightThrottleHigh;
        public static final double RightThrottleLow = isC1 ? C1RightThrottleLow : C2RightThrottleLow;
    }
}
