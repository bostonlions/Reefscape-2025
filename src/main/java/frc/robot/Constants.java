// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Map;

import static java.util.Map.entry;

import frc.robot.drivers.CustomXboxController.Axis;

import static frc.robot.subsystems.SwerveDrive.getSwerveModulePos;

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
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class SwerveConstants {
        // Reductions by which drive speed gets divided
        public static final double strafeReduction = 4.;
        public static final double precisionReduction = 8.;

        // Drivetrain Constants
        public static final double trackWidth = Units.inchesToMeters(24.25);
        public static final double wheelBase = Units.inchesToMeters(24.25);

        // MODULE CANCODER ANGLE OFFSETS - TODO: update these for new swerve drive system
        // To calibrate:
        // - Tip the robot up on its back side
        // - Align the bevel gears to the right side (from lookers perspective) on all the wheels.
        // - Make sure all the wheels are in line, then record CANcoder offset values (in degrees)
        //   from shuffleboard
        public static final double FL_AngleOffset = 335.3027;
        public static final double FR_AngleOffset = 89.1211;
        public static final double BL_AngleOffset = 191.3379;
        public static final double BR_AngleOffset = 73.2129;

        public static final SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration
        > SMConstFactory = new SwerveModuleConstantsFactory<
            TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration
        >()
            .withCouplingGearRatio(50. / 14) // TODO: is this value right?
            // .withDriveMotorGains(SwerveConstants.driveConfig.Slot0) // TODO: do we need this? -- i think initial configs makes this not needed
            .withDriveMotorGearRatio(6.3)
            .withDriveMotorInitialConfigs(
                new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(65)
                    .withSupplyCurrentLowerLimit(62)
                    .withSupplyCurrentLowerTime(0.1))
                .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12.)
                    .withPeakReverseVoltage(-12.))
                .withSlot0(new Slot0Configs()
                    .withKP(0.03)
                    .withKI(0.)
                    .withKD(0.)
                    .withKV(0.124))
                .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
                .withOpenLoopRamps(new OpenLoopRampsConfigs()
                    .withDutyCycleOpenLoopRampPeriod(0.25)
                    .withVoltageOpenLoopRampPeriod(0.25))
            )
            .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
            .withEncoderInitialConfigs(
                new CANcoderConfiguration()
                    .withMagnetSensor(new MagnetSensorConfigs()
                    .withAbsoluteSensorDiscontinuityPoint(1.)
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive))
            )
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.RemoteCANcoder)
            // .withSteerMotorGains(SwerveConstants.angleConfig.Slot0) // TODO: do we need this? -- i think initial configs makes this not needed
            .withSteerMotorGearRatio(150. / 7)
            .withSteerMotorInitialConfigs(
                new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(40)
                    .withSupplyCurrentLowerLimit(25)
                    .withSupplyCurrentLowerTime(0.1))
                .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12.)
                    .withPeakReverseVoltage(-12.))
                .withSlot0(new Slot0Configs()
                    .withKP(70.)
                    .withKI(0.)
                    .withKD(0.5)
                    .withKS(0.1)
                    .withKV(1.91)
                    .withKA(0.)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
                .withMotorOutput(new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.Clockwise_Positive))
            )
            .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
            .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
            .withSpeedAt12Volts(5.02)
            .withWheelRadius(Units.inchesToMeters(2));
    }

    public static final class AutonConstants {
        public static final RobotConfig pathPlannerConfig = new RobotConfig(
            57.,
            3.873,
            new ModuleConfig(
                SwerveConstants.SMConstFactory.WheelRadius,
                SwerveConstants.SMConstFactory.SpeedAt12Volts * 0.85,
                1.15,

                // .withReduction here is VERY IMPORTANT! Autonomous drive WILL NOT WORK WITHOUT IT
                DCMotor.getKrakenX60(1).withReduction(
                    SwerveConstants.SMConstFactory.DriveMotorGearRatio
                ),

                111.,
                4
            ),
            new Translation2d(
                getSwerveModulePos(4, Axis.X),
                getSwerveModulePos(4, Axis.Y)
            ),
            new Translation2d(
                getSwerveModulePos(2, Axis.X),
                getSwerveModulePos(2, Axis.Y)
            ),
            new Translation2d(
                getSwerveModulePos(3, Axis.X),
                getSwerveModulePos(3, Axis.Y)
            ),
            new Translation2d(
                getSwerveModulePos(1, Axis.X),
                getSwerveModulePos(1, Axis.Y)
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
        public static final double gearRatio = 5.;
        public static final double wheelCircumference = 0.12; // 24 teeth x 5mm belt tooth pitch - 1.625" * PI is ~0.129m
        public static final double positionError = Units.inchesToMeters(0.25);
        public static final double bottomLimitTorque = 600.; // trying to see if making these huge fixes it
        public static final double zeroingLimitTorque = 30.; // trying to see if making these huge fixes it
        public static final double topLimitTorque = 1000.;
        /** Velocity tolerance within which the elevator might be stopped */
        public static final double limitVelocity = 0.1;
        /** Meters from target to consider movement complete */
        public static final double heightTolerance = 0.005;
        public static final double resetDutyCycle = 0.3;

        public enum Position { MIN, LOAD, L2, L3, L4, BARGE, MAX, MANUAL }

        /** Heights in meters */
        public static final Map<Position, Double> heights = Map.ofEntries(
            entry(Position.MIN, 0.06), // increased by 0.06
            entry(Position.LOAD, 0.063), // increased by 0.06
            entry(Position.L2, 0.36), // .36 home: .41
            entry(Position.L3, 0.75), // .75 home: .82
            entry(Position.L4, 1.38), // .402 home: 1.41
            entry(Position.BARGE, 1.48),
            entry(Position.MAX, 1.485),
            entry(Position.MANUAL, 0.) // not targeting a set position; controlled manually with trimmer
        );
        /** These are the positions you can access with step up and down */
        public static final List<Position> positionOrder = List.of(
            Position.LOAD, Position.L2, Position.L3, Position.L4, Position.BARGE
        );

        public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLowerLimit(20)
                .withSupplyCurrentLowerTime(0.1))
            .withSlot0(new Slot0Configs()
                .withKP(0.6)
                .withKI(0.1)
                .withKD(0.)
                .withKV(0.))
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
        public static final double limitVelocity = 1.;
        public static final double extensionTolerance = 0.1;

        public static final double extendSpeed = 2.5;
        public static final double climbSpeed = 3.5;
        public static final double nudgeSpeed = 2.1;
        /** Seconds to wait at DROP before climbing */
        public static final double climbDelay = 0.5;
        public static final double footReleaseDelay = 2.;
        public static final double footReleaseRotations = 0.6;

        public enum Position { MIN, CLIMBED, STOW, LATCH, DROP, MAX, MANUAL }

        /** Values are in winch rotations */
        public static final Map<Position, Double> extensions = Map.ofEntries(
            entry(Position.MIN, -4.5),
            entry(Position.CLIMBED, -4.11),
            entry(Position.STOW, 0.), // front of prop hitting circle at front of hook
            entry(Position.LATCH, 0.30), // .99 front of prop hitting back of hook top plate
            entry(Position.DROP, 1.5),
            entry(Position.MAX, 3.)
        );

        public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLowerLimit(30)
                .withSupplyCurrentLowerTime(0.1))
            .withSlot0(new Slot0Configs()
                .withKP(8.) // 0.02
                .withKI(0.)
                .withKD(0.)
                .withKV(0.))
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
        public static final double loadSpeed = 0.7;
        /** If this is 0 we never break from case statement in coral.java */
        public static final double extraLoadRotations = 0.02;
        public static final double unloadSpeed = 0.35;
        public static final double extraUnloadRotations = 0.2;

        public static TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLowerLimit(10)
                .withSupplyCurrentLowerTime(0.1))
            .withSlot0(new Slot0Configs()
                .withKP(0.6)
                .withKI(0.)
                .withKD(0.)
                .withKV(0.))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(0.5)
                .withMotionMagicExpo_kA(0.3)
                .withMotionMagicAcceleration(1.))
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive));
    }

    public static final class AlgaeConstants {
        public static final double intakeSuction = -0.1;

        public static final double angleGearRatio = (57. / 15) * 5;
        public static final double driveGearRatio = 2.;

        /** Hardly a nudge anymore; nudge is now to push balls into the processor */
        public static final double nudgeSpeed = 18.;

        /** Should be the amount of SECONDS it takes to stop */
        public static final double extraGroundIntakeTime = .07; // .009
        public static final double groundIntakeSpeed = 25.; // was 0.5

        /** Should be the amount of SECONDS it takes to stop */
        public static final double extraReefIntakeTime = 0.8;
        public static final double reefIntakeSpeed = 20.;

        /** Should be the amount of ROTATIONS it takes to stop */
        public static final double extraProcessorUnloadRotations = 36.;
        public static final double processorUnloadSpeed = 18.;

        /** Should be the amount of ROTATIONS it takes to stop */
        public static final double extraBargeUnloadRotations = 15.;
        public static final double bargeUnloadSpeed = 18.;

        /**
         * Cancoder offset should be set to reading in degrees at exactly horizontal.
         * To get that value, set cancoderOffset to 0, load the code to the robot, and
         * look at smart dashboard for the CANCoder Position when it is horizontal
         */
        public static final double cancoderOffset = -27.;

        public enum Position {
            MIN, STOW_DOWN, GROUND_INTAKE, LOADED_DOWN, PROCESSOR, STOW_UP, REEF, LOADED_UP, BARGE, MAX
        }

        /** Angles in degrees from horizontal */
        public static final Map<Position, Double> angles = Map.ofEntries(
            entry(Position.MIN, -91.),
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
                .withKP(1.65) // 6.5 w 9
                .withKI(0.04) // .09 w 9
                .withKD(0.012)
                .withKV(0.)
                .withKA(0.)
                .withKS(0.)
                .withKG(0.))
            .withMotionMagic(new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(8.3) // 42 w9
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
        public static final double triggerThreshold = 0.2;
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

        public static final double LeftThrottleZero = isC1 ? -0.125 : -0.023438;
        public static final double LeftYawZero = isC1 ? 0.039370 : -0.078125;
        public static final double LeftThrottleHigh = isC1 ? 0.787402 : 0.834646;
        public static final double LeftThrottleLow = isC1 ? 0.968750 : 0.867188;
        public static final double LeftYawHigh = isC1 ? 0.86612 : 0.748031;
        public static final double LeftYawLow = isC1 ? 0.77338 : 0.890625;
        public static final double RightThrottleZero = isC1 ? 0.055118 : -0.054688; // high 0.007874 (for C2 one)
        public static final double RightYawZero = isC1 ? 0.055118 : 0.062992;
        public static final double RightYawHigh = isC1 ? 0.866142 : 0.866142;
        public static final double RightYawLow = isC1 ? 0.765625 : 0.664063;
        public static final double RightThrottleHigh = isC1 ? 0.732283 : 0.669291;
        public static final double RightThrottleLow = isC1 ? 0.601563 : 0.664063;
    }
}
