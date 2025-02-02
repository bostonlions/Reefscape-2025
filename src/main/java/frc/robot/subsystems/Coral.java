package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Ports;
import frc.robot.lib.drivers.BeamBreak;
import frc.robot.Constants.CoralConstants;

public class Coral extends SubsystemBase {
    private static Coral mInstance;
    private TalonFX mMotor;
    private BeamBreak mBeamBreak;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private enum State {
        IDLE, LOADING_NO_CORAL, LOADING_WITH_CORAL, LOADED, UNLOADING_WITH_CORAL, UNLOADING_NO_CORAL
    }

    public static Coral getInstance() {
        if (mInstance == null) mInstance = new Coral();
        return mInstance;
    }

    private Coral() {
        mMotor = new TalonFX(Ports.CORAL_DRIVE, Ports.CANBUS_OPS);
        mMotor.getConfigurator().apply(CoralConstants.motorConfig);
        setWantNeutralBrake(true);
        mMotor.setPosition(0);
        mBeamBreak = new BeamBreak(Ports.CORAL_BEAM_BREAK);
    }

    public void disable() {
        // TODO
    }

    public void setWantNeutralBrake(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mMotor.setNeutralMode(mode);
    }

    public void setSetpointMotionMagic(double speed) {
        mPeriodicIO.demand = speed * CoralConstants.gearRatio;
        mMotor.setControl(new MotionMagicVelocityDutyCycle(mPeriodicIO.demand));
    }

    /** To start or interrupt a load or unload upon a button push */
    public void activateCoral() {
        if (mPeriodicIO.state == State.IDLE) {
            setSetpointMotionMagic(CoralConstants.loadSpeed);
            mPeriodicIO.state = State.LOADING_NO_CORAL;
        } else if (mPeriodicIO.state == State.LOADED) {
            setSetpointMotionMagic(CoralConstants.unloadSpeed);
            mPeriodicIO.state = State.UNLOADING_WITH_CORAL;
        } else {
            setSetpointMotionMagic(0);
            mPeriodicIO.state = mBeamBreak.get() ? State.LOADED : State.IDLE;
        }
    }

    public double getAngleDeg() {
        return mPeriodicIO.position_degrees;
    }

    public double getTorqueCurrent() {
        return mPeriodicIO.current;
    }

    public static class PeriodicIO {
        // Inputs
        public double position_degrees = 0.0;
        public double velocity_rps = 0.0;
        public double current = 0.0;
        public double output_voltage = 0.0;

        // Outputs
        public double demand = 0;
        public long stopTime;
        public State state = State.LOADED;
    }

    @Override
    public void periodic() {
        switch (mPeriodicIO.state) {
            // error correction:
            case IDLE:
                if (mBeamBreak.get()) mPeriodicIO.state = State.LOADED;
                break;
            case LOADED:
                if (!mBeamBreak.get()) mPeriodicIO.state = State.IDLE;
                break;

            case LOADING_NO_CORAL:
                if (mBeamBreak.get()) {
                    mPeriodicIO.stopTime = System.currentTimeMillis() + (long)(1000 *
                        (CoralConstants.extraLoadRotations / CoralConstants.loadSpeed));
                    mPeriodicIO.state = State.LOADING_WITH_CORAL;
                }
                if (CoralConstants.extraLoadRotations > 0) break; // we wanna check the next case without
                // waiting for next periodic loop, just if no wait time after the load
            case LOADING_WITH_CORAL:
                if (System.currentTimeMillis() >= mPeriodicIO.stopTime) {
                    setSetpointMotionMagic(0);
                    mPeriodicIO.state = State.LOADED;
                } break;
            case UNLOADING_WITH_CORAL:
                if (!mBeamBreak.get()) {
                    mPeriodicIO.stopTime = System.currentTimeMillis() + (long)(1000 *
                        (CoralConstants.extraUnloadRotations / CoralConstants.unloadSpeed));
                    mPeriodicIO.state = State.UNLOADING_NO_CORAL;
                }
                if (CoralConstants.extraUnloadRotations > 0) break;
            case UNLOADING_NO_CORAL:
                if (System.currentTimeMillis() >= mPeriodicIO.stopTime) {
                    setSetpointMotionMagic(0);
                    mPeriodicIO.state = State.IDLE;
                } break;
        }

        mPeriodicIO.position_degrees = mMotor.getRotorPosition().getValue().in(Units.Degrees) / CoralConstants.gearRatio;
        mPeriodicIO.current = mMotor.getTorqueCurrent().getValue().in(Units.Amps);
        mPeriodicIO.output_voltage = mMotor.getMotorVoltage().getValue().in(Units.Volts);
        mPeriodicIO.velocity_rps = mMotor.getVelocity().getValue().in(Units.RotationsPerSecond) / CoralConstants.gearRatio;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Coral Angle (degrees)", () -> mPeriodicIO.position_degrees, null);
        builder.addDoubleProperty("Coral Motor Rotations", () -> mMotor.getRotorPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Coral Demand", () -> mPeriodicIO.demand, null);
        builder.addDoubleProperty("Coral Velocity rps", () -> mPeriodicIO.velocity_rps, null);
        builder.addDoubleProperty("Coral Volts", () -> mPeriodicIO.output_voltage, null);
        builder.addDoubleProperty("Coral Current", () -> mPeriodicIO.current, null);
        builder.addStringProperty("Coral State", () -> mPeriodicIO.state.toString(), null);
        builder.addBooleanProperty("Activate Coral", () -> false, (v) -> {if(v) activateCoral();});
        builder.setSafeState(this::disable);
        builder.setActuator(true);
    }
}
