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

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;

import frc.robot.lib.Util.Conversions;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public class Constants {
    // Disables extra smart dashboard outputs that slow down the robot
    public static final boolean disableExtraTelemetry = false;

    public static final class SwerveConstants {
        // robot loop time - but only used now by swerve
        public static final double kLooperDt = 0.02;

        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(24.25);
        public static final double wheelBase = Units.inchesToMeters(24.25);

        public static final double wheelDiameter = Units.inchesToMeters(3.85);

        // can tune this value by driving a certain distance and multiplying a const to
        // fix the error
        public static final double driveGearRatio = 4.7628; // ((5.3 / 1.07) / 1.04) ?
        public static final double angleGearRatio = 150./7; // 21.4285714

        /* Swerve Profiling Values */
        public static final double maxSpeed = 2.; // was 4.8. meters per second MAX : 5.02 m/s
        public static final double maxAccel = 1.;
        public static final double maxAngularVelocity = 2.; //was 8.0
        public static final double maxAngularAccel = 1.;

        // Max out at 85% to make sure speeds are attainable (4.6 mps)
        public static final double maxAttainableSpeed = maxSpeed * 0.85;

        public static final TalonFXConfiguration driveConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(65)
                .withSupplyCurrentLowerLimit(62)
                .withSupplyCurrentLowerTime(0.1))
            .withVoltage(new VoltageConfigs()
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0))
            .withSlot0(new Slot0Configs()
                .withKP(4.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKV(12.0 / Conversions.MPSToRPS(maxSpeed, wheelDiameter * Math.PI, driveGearRatio)))
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
                .withPeakForwardVoltage(12.0)
                .withPeakReverseVoltage(-12.0))
            .withSlot0(new Slot0Configs()
                .withKP(0.6)
                .withKI(0.0)
                .withKD(0.0)
                .withKV(0.0))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive));

        public static final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(1.0)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

        public static class KinematicLimits {
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
            // maxSpeed, Double.MAX_VALUE, maxAngularVelocity, Double.MAX_VALUE
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

    // For DriveMotionPlanner - what is snap?
    public static final class SnapConstants {
        public static final double kP = 6.0;
        public static final double kI = 0.5;
        public static final double kD = 0.2;
        public static final double snapTimeout = 0.25;
        public static final double snapEpsilon = 1.0;
    }

    // For DriveMotionPlanner
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

    /*** SUBSYSTEM CONSTANTS ***/

    public static final class ElevatorConstants {
        public static final double gearRatio = 9.;
        public static final double wheelCircumference = 0.12; // 24 teeth x 5mm belt tooth pitch - 1.625" * PI is ~0.129m
        public static final double positionError = Conversions.inchesToMeters(0.25);
        public static final double limitTorque = 30;
        public static final double limitVelocity = 0.1;
        public static final double heightTolerance = 0.005; // meters from target to consider movement complete

        // Heights in meters
        // TODO: values are placeholders. Are LOAD, PROCESSOR, and L1 all the same?
        public enum Position { MIN, LOAD, PROCESSOR, L1, L2, L3, L4, BARGE, MAX, MANUAL }
        public static final Map<Position, Double> heights = Map.ofEntries(
            entry(Position.MIN, 0.),
            entry(Position.LOAD, 0.001),
            entry(Position.PROCESSOR, 0.01),
            entry(Position.L1, 0.01),
            entry(Position.L2, 0.2),
            entry(Position.L3, 0.5),
            entry(Position.L4, 1.3),
            entry(Position.BARGE, 1.43),
            entry(Position.MAX, 1.431),
            entry(Position.MANUAL, 0.) // not targeting a set position; controlled manually from Shuffleboard
        );
        // These are the positions you can access with step up and down
        public static final List<Position> positionOrder = List.of(
            Position.LOAD, Position.PROCESSOR, Position.L1, Position.L2, Position.L3, Position.L4, Position.BARGE
        );

        public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLowerLimit(20)
                .withSupplyCurrentLowerTime(0.1))
            .withSlot0(new Slot0Configs()
                .withKP(0.6)
                .withKI(0.0)
                .withKD(0.0)
                .withKV(0.0))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(14)
                .withMotionMagicExpo_kA(0.3)
                .withMotionMagicAcceleration(30))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));
    }

    public static final class ClimberHookConstants {
        public static final double gearRatio = 4.6*25;
        public static final double limitTorque = 100.;
        public static final double limitVelocity = 0.1;
        public static final double extensionTolerance = 0.5;

        public static final double fastSpeed = 100;
        public static final double slowSpeed = 10;

        public enum Position { MIN, IN, OUT, MAX, MANUAL }
        public static final Map<Position, Double> extensions = Map.ofEntries(
            // values are in degrees, but aren't correct yet
            entry(Position.MIN, 0.),
            entry(Position.IN, 10.),
            entry(Position.OUT, 180.),
            entry(Position.MAX, 200.)
        );

        public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLowerLimit(30)
                .withSupplyCurrentLowerTime(0.1))
            .withSlot0(new Slot0Configs()
                .withKP(0.6)
                .withKI(0.0)
                .withKD(0.0)
                .withKV(0.0))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(140)
                .withMotionMagicExpo_kA(0.3)
                .withMotionMagicAcceleration(300))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));
    }

    public static final class CoralConstants {
        public static final double gearRatio = 4;
        public static final double loadSpeed = 0.5;
        public static final double extraLoadRotations = 0;
        public static final double unloadSpeed = 0.5;
        public static final double extraUnloadRotations = 0;

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
                .withMotionMagicCruiseVelocity(140)
                .withMotionMagicExpo_kA(0.3)
                .withMotionMagicAcceleration(300))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));
    }

    public static final class AlgaeConstants {
        public static final double angleGearRatio = 45;
        public static final double driveGearRatio = 4;

        public static final double groundIntakeDemand = 0.5;
        public static final double reefIntakeDemand = 0.5;
        public static final double processorReleaseDemand = 0.5;
        public static final double bargeReleaseDemand = -0.5;

        // Set the cancoder offset to its reading in degrees at exactly horizontal
        public static final double cancoderOffset = 0;

        // Angles - in degrees from horizontal
        // TODO - values are placeholders
        public enum Position { MIN, STOW, PROCESSOR, GROUND_INTAKE, REEF, BARGE, MAX }
        public static final Map<Position, Double> extensions = Map.ofEntries(
            entry(Position.MIN, -100.),
            entry(Position.STOW, -90.),
            entry(Position.PROCESSOR, -75.),
            entry(Position.GROUND_INTAKE, -70.),
            entry(Position.REEF, 90.),
            entry(Position.BARGE, 90.),
            entry(Position.MAX, 120.)
        );

        public static TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration()
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
                .withMotionMagicCruiseVelocity(140)
                .withMotionMagicExpo_kA(0.3)
                .withMotionMagicAcceleration(300))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));

        public static TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration()
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
                .withMotionMagicCruiseVelocity(140)
                .withMotionMagicExpo_kA(0.3)
                .withMotionMagicAcceleration(300))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));

        public static final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration()
            .withMagnetSensor(new MagnetSensorConfigs()
                .withAbsoluteSensorDiscontinuityPoint(0.5)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
    }

    public static final class ControllerConstants {
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
