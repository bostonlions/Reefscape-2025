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

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class SwerveConstants {
        /** The speed reduction when drive is in strafe mode; drive speed gets divided by this */
        public static final double strafeReduction = 4.;
        /** The speed reduction when drive is in precision mode; drive speed gets divided by this */
        public static final double precisionReduction = 8.;

        public static final double maxAngularVelocity = 8.;

        // Drivetrain Constants
        private static final double trackWidth = Units.inchesToMeters(24.25);
        private static final double wheelBase = Units.inchesToMeters(24.25);

        /** Robot relative */
        public static final List<Pair<Double, Double>> swerveModulePositions = List.of(
            new Pair<Double, Double>(trackWidth / 2, wheelBase / 2),
            new Pair<Double, Double>(-trackWidth / 2, wheelBase / 2),
            new Pair<Double, Double>(trackWidth / 2, -wheelBase / 2),
            new Pair<Double, Double>(-trackWidth / 2, -wheelBase / 2)
        );

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
                    .withKP(1.2)
                    .withKI(0.)
                    .withKD(0.)
                    .withKS(0.)
                    .withKV(1.)
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
                SwerveConstants.swerveModulePositions.get(3).getFirst(),
                SwerveConstants.swerveModulePositions.get(3).getSecond()
            ),
            new Translation2d(
                SwerveConstants.swerveModulePositions.get(1).getFirst(),
                SwerveConstants.swerveModulePositions.get(1).getSecond()
            ),
            new Translation2d(
                SwerveConstants.swerveModulePositions.get(2).getFirst(),
                SwerveConstants.swerveModulePositions.get(2).getSecond()
            ),
            new Translation2d(
                SwerveConstants.swerveModulePositions.get(0).getFirst(),
                SwerveConstants.swerveModulePositions.get(0).getSecond()
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
        public static final double limitVelocity = 0.1;
        public static final double heightTolerance = 0.005; // meters from target to consider movement complete
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
                .withKP(.6)
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
        public static final double climbDelay = 0.5; // seconds to wait at DROP before climbing
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
                .withKP(8) // 0.02
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
        /** If this is 0 we never break from case statement */
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
        public static final double intakeSuction = -.1;

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
        /**
         * Max says trimming joystick input by a percent is the best way to
         * limit speed; that way the autonomous system doesn't get messed up.
         * <p> Set to 1 for 100% of joystick range
         */
        public static final double kInputClipping = 1.;
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
