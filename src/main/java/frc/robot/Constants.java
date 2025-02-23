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
import edu.wpi.first.math.util.Units;

import frc.robot.subsystems.SwerveDrive;

public final class Constants {
    public static final class SwerveConstants {
        // Drivetrain Constants
        public static final double trackWidth = Units.inchesToMeters(24.25);
        public static final double wheelBase = Units.inchesToMeters(24.25);

        public static final double wheelDiameter = Units.inchesToMeters(4); // was 3.85 but tire is 4 w/tread

        /** Can tune this value by driving a certain distance and multiplying a const to fix the error */
        public static final double driveGearRatio = 6.3; // also tried 6.12 // ((5.3 / 1.07) / 1.04) ?  maybe 4.7628
        public static final double angleGearRatio = 150./7;
        public static final double couplingGearRatio = 14./50; // TODO: check if this value is right

        public static final double maxSpeed = 5.02; // was 4.8 toggled to 2.0 meters per second MAX : 5.02 m/s
        public static final double maxAngularVelocity = 8.; //was 8. toggled to 2.0

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
                .withKP(0.6)
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

        // MODULE CANCODER ANGLE OFFSETS
        // To calibrate:
        // - Tip the robot up on its back side
        // - Align the bevel gears to the right side (from lookers perspective) on all the wheels.
        // - Make sure all the wheels are in line, then record CANcoder offset values (in degrees)
        //   from shuffleboard
        public static final double FL_AngleOffset = 335.3027;
        public static final double FR_AngleOffset = 89.1211;
        public static final double BL_AngleOffset = 191.3379;
        public static final double BR_AngleOffset = 73.2129;
    }

    public static final class AutonConstants {
        public static final RobotConfig pathPlannerConfig = new RobotConfig(
            57.,
            3.873, // TODO: get this right?
            new ModuleConfig(
                SwerveConstants.wheelDiameter/2,
                SwerveConstants.maxAttainableSpeed,
                1.15,

                // .withReduction here is VERY IMPORTANT! Autonomous drive WILL NOT WORK WITHOUT IT
                DCMotor.getKrakenX60(1).withReduction(SwerveConstants.driveGearRatio),

                111.,
                4
            ),
            new Translation2d(
                SwerveDrive.getInstance().swerveModulePositions.get(4).getFirst(),
                SwerveDrive.getInstance().swerveModulePositions.get(4).getSecond()
            ),
            new Translation2d(
                SwerveDrive.getInstance().swerveModulePositions.get(2).getFirst(),
                SwerveDrive.getInstance().swerveModulePositions.get(2).getSecond()
            ),
            new Translation2d(
                SwerveDrive.getInstance().swerveModulePositions.get(3).getFirst(),
                SwerveDrive.getInstance().swerveModulePositions.get(3).getSecond()
            ),
            new Translation2d(
                SwerveDrive.getInstance().swerveModulePositions.get(1).getFirst(),
                SwerveDrive.getInstance().swerveModulePositions.get(1).getSecond()
            )
        );

        /** PPHolonomicController is the built in path following controller for holonomic drive trains */
        public static final PPHolonomicDriveController ppHolonomicDriveController =
            new PPHolonomicDriveController(
                new PIDConstants(5., 0., 0.), // Translation PID constants
                new PIDConstants(5., 0., 0.) // Rotation PID constants
            );
    }

    public static final class ElevatorConstants {
        public static final double gearRatio = 9.;
        public static final double wheelCircumference = 0.12; // 24 teeth x 5mm belt tooth pitch - 1.625" * PI is ~0.129m
        public static final double positionError = Units.inchesToMeters(0.25);
        public static final double bottomLimitTorque = 30.;
        public static final double topLimitTorque = 50;
        public static final double limitVelocity = 0.1;
        public static final double heightTolerance = 0.005; // meters from target to consider movement complete
        public static final double resetDutyCycle = 0.1;

        // Heights in meters
        // TODO: values are placeholders. Are LOAD, and L1 all the same?
        public enum Position { MIN, LOAD, L1, L2, L3, L4, BARGE, MAX, MANUAL }
        public static final Map<Position, Double> heights = Map.ofEntries(
            entry(Position.MIN, 0.),
            entry(Position.LOAD, 0.003),
            entry(Position.L1, 0.12),
            entry(Position.L2, 0.32),
            entry(Position.L3, 0.75),
            entry(Position.L4, 1.37),
            entry(Position.BARGE, 1.43),
            entry(Position.MAX, 1.431),
            entry(Position.MANUAL, 0.) // not targeting a set position; controlled manually from Shuffleboard
        );
        // These are the positions you can access with step up and down
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
                .withKP(0.6)
                .withKI(0.0)
                .withKD(0.0)
                .withKV(0.0))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(75)
                .withMotionMagicExpo_kA(0.3)
                .withMotionMagicAcceleration(120))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));
    }

    public static final class ClimberHookConstants {
        public static final double gearRatio = 36.;
        public static final double limitTorque = 100.;
        public static final double limitVelocity = 0.1;
        public static final double extensionTolerance = 0.5;

        public static final double fastSpeed = 100;
        public static final double slowSpeed = 10;
        public static final double nudgeSpeed = 0.5;

        public enum Position { MIN, IN, OUT, MAX, MANUAL }
        public static final Map<Position, Double> extensions = Map.ofEntries(
            // TODO: values are in degrees, but aren't correct yet
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
                .withKP(0.02)
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
        public static final double gearRatio = 4.;
        public static final double loadSpeed = 0.75;
        /** If this is 0 we never break from case statement */
        public static final double extraLoadRotations = 0.03;
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
        public static final double angleGearRatio = (57./15)*5;
        public static final double driveGearRatio = 2.;  // TODO figure out the actual number

        public static final double nudgeSpeed = 3.;

        public static final double extraGroundIntakeTime = .07; //.009 // should be the amount of SECONDS it takes to stop
        public static final double groundIntakeSpeed = 18; //was 0.5

        public static final double extraReefIntakeTime = 0.5; // should be the amount of SECONDS it takes to stop
        public static final double reefIntakeSpeed = 16;

        public static final double extraProcessorUnloadRotations = 1.; // should be the amount of ROTATIONS it takes to stop
        public static final double processorUnloadSpeed = 18.;

        public static final double extraBargeUnloadRotations = 2.; // should be the amount of ROTATIONS it takes to stop
        public static final double bargeUnloadSpeed = 18;

        /**
         * Set this to CANcoder's reading in degrees at exactly horizontal.
         * To get that value, set this to 0, load the code to the robot and
         * look at smart dashboard for the CANcoder position when it is horizontal.
         */
        public static final double cancoderOffset = -27.; // -112.5; // -144.759;  38??

        public enum Position {
            MIN, STOW_DOWN, GROUND_INTAKE, LOADED_DOWN, PROCESSOR, STOW_UP, REEF, LOADED_UP, BARGE, MAX
        }

        /** Angles in degrees from horizontal */
        public static final Map<Position, Double> angles = Map.ofEntries(
            entry(Position.MIN, -82.),
            entry(Position.STOW_DOWN, -82.),
            entry(Position.PROCESSOR, -50.),
            entry(Position.LOADED_DOWN, -63.),
            entry(Position.GROUND_INTAKE, -57.),
            entry(Position.REEF, 50.),
            entry(Position.BARGE, 95.),
            entry(Position.LOADED_UP, 95.),
            entry(Position.STOW_UP, 95.),
            entry(Position.MAX, 115.)
        );

        // /** Small motion range for testing only */
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
                .withKI(0.04) // .09 w 9
                .withKD(0.012)
                .withKV(0.)
                .withKA(0.)
                .withKS(0.)
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
        /**
         * Max says trimming joystick input by a percent is the best way to
         * limit speed; that way the autonomous system doesn't get messed up.
         * <p> Set to 1 for 100% of joystick range
         */
        public static final double kInputClipping = 0.6;

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

        // If not Mambo there are 2 controllers with the same mechanics, but different calibrations
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
