package frc.robot.subsystems;

import java.util.Map;
import static java.util.Map.entry;

import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import frc.robot.Ports;
import frc.robot.lib.drivers.BeamBreak;
import frc.robot.lib.Util;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeConstants.Position;

import static frc.robot.Constants.AlgaeConstants.angleMotorConfig;
import static frc.robot.Constants.AlgaeConstants.driveMotorConfig;
import static frc.robot.Constants.AlgaeConstants.angles;

public class Algae extends SubsystemBase {
    private static Algae mInstance;
    private TalonFX mDriveMotor;
    private TalonFX mAngleMotor;
    private BeamBreak mBeamBreak;
    private CANcoder mCANcoder;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    public enum PositionState { UP, DOWN }
    private enum DriveState {
        IDLE, INTAKE_NO_ALGAE, INTAKE_WITH_ALGAE, LOADED, UNLOADING_WITH_ALGAE, UNLOADING_NO_ALGAE
    }
    private static final Map<DriveState, Position> UP_POSITIONS = Map.ofEntries(
        entry(DriveState.IDLE, Position.STOW_UP),
        entry(DriveState.INTAKE_NO_ALGAE, Position.REEF),
        entry(DriveState.INTAKE_WITH_ALGAE, Position.REEF),
        entry(DriveState.LOADED, Position.LOADED_UP),
        entry(DriveState.UNLOADING_WITH_ALGAE, Position.BARGE),
        entry(DriveState.UNLOADING_NO_ALGAE, Position.BARGE)
    );
    private static final Map<DriveState, Position> DOWN_POSITIONS = Map.ofEntries(
        entry(DriveState.IDLE, Position.STOW_DOWN),
        entry(DriveState.INTAKE_NO_ALGAE, Position.GROUND_INTAKE),
        entry(DriveState.INTAKE_WITH_ALGAE, Position.GROUND_INTAKE),
        entry(DriveState.LOADED, Position.LOADED_DOWN),
        entry(DriveState.UNLOADING_WITH_ALGAE, Position.PROCESSOR),
        entry(DriveState.UNLOADING_NO_ALGAE, Position.PROCESSOR)
    );
    private static final Map<DriveState, Double> UP_SPEEDS = Map.ofEntries(
        entry(DriveState.IDLE, 0.),
        entry(DriveState.INTAKE_NO_ALGAE, -AlgaeConstants.reefIntakeSpeed),
        entry(DriveState.INTAKE_WITH_ALGAE, -AlgaeConstants.reefIntakeSpeed),
        entry(DriveState.LOADED, 0.),
        entry(DriveState.UNLOADING_WITH_ALGAE, AlgaeConstants.bargeUnloadSpeed),
        entry(DriveState.UNLOADING_NO_ALGAE, AlgaeConstants.bargeUnloadSpeed)
    );
    private static final Map<DriveState, Double> DOWN_SPEEDS = Map.ofEntries(
        entry(DriveState.IDLE, 0.),
        entry(DriveState.INTAKE_NO_ALGAE, AlgaeConstants.groundIntakeSpeed),
        entry(DriveState.INTAKE_WITH_ALGAE, AlgaeConstants.groundIntakeSpeed),
        entry(DriveState.LOADED, 0.),
        entry(DriveState.UNLOADING_WITH_ALGAE, AlgaeConstants.processorUnloadSpeed),
        entry(DriveState.UNLOADING_NO_ALGAE, AlgaeConstants.processorUnloadSpeed)
    );

    public static Algae getInstance() {
        if (mInstance == null) mInstance = new Algae();
        return mInstance;
    }

    private Algae() {
        mBeamBreak = new BeamBreak(Ports.ALGAE_BEAMBREAK);

        mCANcoder = new CANcoder(Ports.ALGAE_CANCODER, Ports.CANBUS_OPS);
        mCANcoder.getConfigurator().apply(AlgaeConstants.cancoderConfig);

        mDriveMotor = new TalonFX(Ports.ALGAE_DRIVE, Ports.CANBUS_OPS);

        mAngleMotor = new TalonFX(Ports.ALGAE_ANGLE, Ports.CANBUS_OPS);
        angleMotorConfig.Feedback.FeedbackRemoteSensorID = mCANcoder.getDeviceID();
        angleMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        angleMotorConfig.Feedback.SensorToMechanismRatio = 1.;
        angleMotorConfig.Feedback.RotorToSensorRatio = AlgaeConstants.gearRatio;
        setConfigs();
        // mAngleMotor.setPosition(getAdjustedCancoderAngle() * AlgaeConstants.gearRatio / 360);
        initTrimmer();
    }

    public void setConfigs() {
        mDriveMotor.getConfigurator().apply(driveMotorConfig);
        mDriveMotor.setPosition(0);
        mAngleMotor.getConfigurator().apply(angleMotorConfig);
    }

    public void disable() {
        // TODO
    }

    public void setAngleSetpoint(double angle) {
        mPeriodicIO.angleSetpoint = angle;
        mPeriodicIO.C_demand = angle / 360;
        mDriveMotor.setControl(new MotionMagicVelocityDutyCycle(0));
        // mDriveMotor.setControl(new Follower(Ports.ALGAE_ANGLE, true));
        mAngleMotor.setControl(new MotionMagicDutyCycle(mPeriodicIO.C_demand));
    }

    public void setDriveSpeed(double rps) {
        mPeriodicIO.D_demand = rps * AlgaeConstants.gearRatio;
        mDriveMotor.setControl(new MotionMagicVelocityDutyCycle(mPeriodicIO.D_demand));
        // TODO - do we need to explicitly decouple from angle motor or is a new setControl enough?
    }

    public void setPosition(PositionState newPosition) {
        if (newPosition == mPeriodicIO.requestedPosition) return;
        mPeriodicIO.requestedPosition = newPosition;
        setState();
    }

    public void toggleDrive() {
        switch(mPeriodicIO.driveState) {
            case IDLE:
            case UNLOADING_NO_ALGAE:
                mPeriodicIO.driveState = DriveState.INTAKE_NO_ALGAE;
                break;
            case INTAKE_NO_ALGAE:
                mPeriodicIO.driveState = DriveState.IDLE;
                break;
            case INTAKE_WITH_ALGAE:
            case LOADED:
                mPeriodicIO.driveState = DriveState.UNLOADING_WITH_ALGAE;
                break;
            case UNLOADING_WITH_ALGAE:
                mPeriodicIO.driveState = DriveState.LOADED;
        }
        setState();
    }

    public void setState() {
        boolean isUp = mPeriodicIO.requestedPosition == PositionState.UP;
        mPeriodicIO.targetPosition = (isUp ? UP_POSITIONS : DOWN_POSITIONS).get(mPeriodicIO.driveState);
        mPeriodicIO.targetSpeed = (isUp ? UP_SPEEDS : DOWN_SPEEDS).get(mPeriodicIO.driveState);
        setAngleSetpoint(angles.get(mPeriodicIO.targetPosition));
        if (mPeriodicIO.targetSpeed != 0) setDriveSpeed(mPeriodicIO.targetSpeed);
    }

    public double getAdjustedCancoderAngle() {
        return Util.placeInAppropriate0To360Scope(
            0, mCANcoder.getPosition().getValueAsDouble() * 360  // - AlgaeConstants.cancoderOffset
        );
    }

    private static final class PeriodicIO {
        // Inputs - cancoder motor
        public double C_position_degrees = 0.0;
        public double C_velocity_rps = 0.0;
        public double C_current = 0.0;
        public double C_output_voltage = 0.0;

        // Inputs - drive motor
        public double D_position_degrees = 0.0;
        public double D_velocity_rps = 0.0;
        public double D_current = 0.0;
        public double D_output_voltage = 0.0;

        // Outputs
        public double C_demand = 0;
        public double D_demand = 0;
        public PositionState requestedPosition = PositionState.DOWN;
        public Position targetPosition = Position.STOW_DOWN;
        public double targetSpeed = 0;
        public DriveState driveState = DriveState.IDLE;
        public long stopTime;
        public double angleSetpoint = 0;
        public double adjustedCancoderAngle = 0;
    }

    @Override
    public void periodic() {
        boolean isUp = mPeriodicIO.requestedPosition == PositionState.UP;
        switch (mPeriodicIO.driveState) {
            case INTAKE_NO_ALGAE:
                if (mBeamBreak.get()) {
                    mPeriodicIO.stopTime = System.currentTimeMillis() + (long)(1000 *
                        (isUp ?
                            (AlgaeConstants.extraReefIntakeRotations / AlgaeConstants.reefIntakeSpeed) :
                            (AlgaeConstants.extraGroundIntakeRotations / AlgaeConstants.groundIntakeSpeed)
                        )
                    );
                    mPeriodicIO.driveState = DriveState.INTAKE_WITH_ALGAE;
                }
                if ((isUp ? AlgaeConstants.extraReefIntakeRotations : AlgaeConstants.extraGroundIntakeRotations) > 0) {
                    // we wanna check the next case without waiting for next periodic loop,
                    // just if no wait time after the ground intake
                    break;
                }
            case INTAKE_WITH_ALGAE:
                if (System.currentTimeMillis() >= mPeriodicIO.stopTime) {
                    mPeriodicIO.driveState = DriveState.LOADED;
                    setState();
                } break;
            case UNLOADING_WITH_ALGAE:
                if (!mBeamBreak.get()) {
                    mPeriodicIO.stopTime = System.currentTimeMillis() + (long)(1000 *
                        (isUp ?
                            (AlgaeConstants.extraBargeUnloadRotations / AlgaeConstants.bargeUnloadSpeed) :
                            (AlgaeConstants.extraProcessorUnloadRotations / AlgaeConstants.processorUnloadSpeed)
                        )
                    );
                    mPeriodicIO.driveState = DriveState.UNLOADING_NO_ALGAE;
                }
                if ((isUp ? AlgaeConstants.extraBargeUnloadRotations : AlgaeConstants.extraProcessorUnloadRotations) > 0) {
                    // we wanna check the next case without waiting for next periodic loop,
                    // just if no wait time after the ground intake
                    break;
                }
            case UNLOADING_NO_ALGAE:
                if (System.currentTimeMillis() >= mPeriodicIO.stopTime) {
                    mPeriodicIO.driveState = DriveState.IDLE;
                    setState();
                }
                break;

            // error correction:
            case IDLE:
                if (mBeamBreak.get()) mPeriodicIO.driveState = DriveState.LOADED;
                break;
            case LOADED:
                if (!mBeamBreak.get()) mPeriodicIO.driveState = DriveState.IDLE;
                break;

            default: break;
        }

        mPeriodicIO.C_position_degrees = mAngleMotor.getRotorPosition().getValue().in(Units.Degrees) / AlgaeConstants.gearRatio;
        mPeriodicIO.C_current = mAngleMotor.getTorqueCurrent().getValue().in(Units.Amps);
        mPeriodicIO.C_output_voltage = mAngleMotor.getMotorVoltage().getValue().in(Units.Volts);
        mPeriodicIO.C_velocity_rps = mAngleMotor.getVelocity().getValue().in(Units.RotationsPerSecond) / AlgaeConstants.gearRatio;

        mPeriodicIO.D_position_degrees = mDriveMotor.getRotorPosition().getValue().in(Units.Degrees) / AlgaeConstants.gearRatio;
        mPeriodicIO.D_current = mDriveMotor.getTorqueCurrent().getValue().in(Units.Amps);
        mPeriodicIO.D_output_voltage = mDriveMotor.getMotorVoltage().getValue().in(Units.Volts);
        mPeriodicIO.D_velocity_rps = mDriveMotor.getVelocity().getValue().in(Units.RotationsPerSecond) / AlgaeConstants.gearRatio;

        mPeriodicIO.adjustedCancoderAngle = getAdjustedCancoderAngle();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Algae");
        builder.setSafeState(this::disable);
        builder.setActuator(true);

        builder.addDoubleProperty("Angle motor degrees", () -> mPeriodicIO.C_position_degrees, null);
        builder.addDoubleProperty("Angle motor current", () -> mPeriodicIO.C_current, null);
        builder.addDoubleProperty("Angle motor voltage", () -> mPeriodicIO.C_output_voltage, null);
        builder.addDoubleProperty("Angle motor speed", () -> mPeriodicIO.C_velocity_rps, null);
        builder.addDoubleProperty("Angle motor demand", () -> mPeriodicIO.C_demand, null);
        builder.addDoubleProperty("Drive motor degrees", () -> mPeriodicIO.D_position_degrees, null);
        builder.addDoubleProperty("Drive motor current", () -> mPeriodicIO.D_current, null);
        builder.addDoubleProperty("Drive motor voltage", () -> mPeriodicIO.D_output_voltage, null);
        builder.addDoubleProperty("Drive motor speed", () -> mPeriodicIO.D_velocity_rps, null);
        builder.addDoubleProperty("Drive motor demand", () -> mPeriodicIO.D_demand, null);
        builder.addDoubleProperty("CANCODER Position", () -> mPeriodicIO.adjustedCancoderAngle, null);
        builder.addDoubleProperty("CANCODER Raw Position", () -> mCANcoder.getAbsolutePosition().getValueAsDouble() * 360, null);
        builder.addDoubleProperty("Angle setpoint", () -> mPeriodicIO.angleSetpoint, null);
        builder.addStringProperty("Target PositionState", () -> mPeriodicIO.requestedPosition.toString(), null);
        builder.addStringProperty("Target DriveState", () -> mPeriodicIO.driveState.toString(), null);
        builder.addStringProperty("Target Position", () -> mPeriodicIO.targetPosition.toString(), null);
        builder.addDoubleProperty("Target speed", () -> mPeriodicIO.targetSpeed, null);
    }

    public void initTrimmer() {
        Trimmer trimmer = Trimmer.getInstance();
        trimmer.add(
            "Algae",
            "Angle+-2",
            () -> mPeriodicIO.angleSetpoint,
            (up) -> {setAngleSetpoint(mPeriodicIO.angleSetpoint + (up ? 2 : -2));}
        );

        trimmer.add(
            "Algae",
            "Angle kP",
            () -> angleMotorConfig.Slot0.kP,
            (up) -> {angleMotorConfig.Slot0.kP = Trimmer.increment(angleMotorConfig.Slot0.kP, 0.01, 0.2, up); setConfigs();}
        );
        trimmer.add(
            "Algae",
            "Angle kI",
            () -> angleMotorConfig.Slot0.kI,
            (up) -> {angleMotorConfig.Slot0.kI = Trimmer.increment(angleMotorConfig.Slot0.kI, 0.01, 0.2, up); setConfigs();}
        );
        trimmer.add(
            "Algae",
            "Angle kD",
            () -> angleMotorConfig.Slot0.kD,
            (up) -> {angleMotorConfig.Slot0.kD = Trimmer.increment(angleMotorConfig.Slot0.kD, 0.01, 0.2, up); setConfigs();}
        );
        trimmer.add(
            "Algae",
            "Angle Velocity",
            () -> angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity,
            (up) -> {angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Trimmer.increment(angleMotorConfig.MotionMagic.MotionMagicCruiseVelocity, 0.01, 0.2, up); setConfigs();}
        );
        trimmer.add(
            "Algae",
            "Angle Accel",
            () -> angleMotorConfig.MotionMagic.MotionMagicAcceleration,
            (up) -> {angleMotorConfig.MotionMagic.MotionMagicAcceleration = Trimmer.increment(angleMotorConfig.MotionMagic.MotionMagicAcceleration, 0.01, 0.2, up); setConfigs();}
        );

        trimmer.add(
            "Algae",
            "Drive kP",
            () -> driveMotorConfig.Slot0.kP,
            (up) -> {driveMotorConfig.Slot0.kP = Trimmer.increment(driveMotorConfig.Slot0.kP, 0.01, 0.2, up); setConfigs();}
        );
        trimmer.add(
            "Algae",
            "Drive kI",
            () -> driveMotorConfig.Slot0.kI,
            (up) -> {driveMotorConfig.Slot0.kI = Trimmer.increment(driveMotorConfig.Slot0.kI, 0.01, 0.2, up); setConfigs();}
        );
        trimmer.add(
            "Algae",
            "Drive kD",
            () -> driveMotorConfig.Slot0.kD,
            (up) -> {driveMotorConfig.Slot0.kD = Trimmer.increment(driveMotorConfig.Slot0.kD, 0.01, 0.2, up); setConfigs();}
        );
        trimmer.add(
            "Algae",
            "Drive Velocity",
            () -> driveMotorConfig.MotionMagic.MotionMagicCruiseVelocity,
            (up) -> {driveMotorConfig.MotionMagic.MotionMagicCruiseVelocity = Trimmer.increment(driveMotorConfig.MotionMagic.MotionMagicCruiseVelocity, 0.01, 0.2, up); setConfigs();}
        );
        trimmer.add(
            "Algae",
            "Drive Accel",
            () -> driveMotorConfig.MotionMagic.MotionMagicAcceleration,
            (up) -> {driveMotorConfig.MotionMagic.MotionMagicAcceleration = Trimmer.increment(driveMotorConfig.MotionMagic.MotionMagicAcceleration, 0.01, 0.2, up); setConfigs();}
        );
    }
}
