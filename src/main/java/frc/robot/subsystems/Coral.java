package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
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
        mPeriodicIO.loadSpeed = CoralConstants.loadSpeed;
        mPeriodicIO.unloadSpeed = CoralConstants.unloadSpeed;
        mPeriodicIO.extraLoadRotations = CoralConstants.extraLoadRotations;
        mPeriodicIO.extraUnloadRotations = CoralConstants.extraUnloadRotations;
        initTrimmer();
    }

    public void disable() {
        // TODO
    }

    public void setWantNeutralBrake(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mMotor.setNeutralMode(mode);
    }

    public void setSetpoint(double speed) {
        mPeriodicIO.demand = speed; // * CoralConstants.gearRatio;
        // mMotor.setControl(new MotionMagicVelocityDutyCycle(mPeriodicIO.demand));
        if (speed == 0) mMotor.setControl(new NeutralOut());
        else mMotor.setControl(new DutyCycleOut(speed));
    }

    /** To start or interrupt a load or unload upon a button push */
    public void activateCoral() {
        System.out.println("activateCoral - mPeriodicIO.state: " + mPeriodicIO.state); // a helpful tool for debugging


        if (mPeriodicIO.state == State.IDLE) {
            setSetpoint(mPeriodicIO.loadSpeed);
            mPeriodicIO.state = State.LOADING_NO_CORAL;
        } else if (mPeriodicIO.state == State.LOADED) {
            setSetpoint(mPeriodicIO.unloadSpeed);
            mPeriodicIO.state = State.UNLOADING_WITH_CORAL;
        } else {
            setSetpoint(0);
            mPeriodicIO.state = mBeamBreak.get() ? State.LOADED : State.IDLE;
        }

    }

    public double getAngleDeg() {
        return mPeriodicIO.position_degrees;
    }

    public double getTorqueCurrent() {
        return mPeriodicIO.current;
    }

    private static final class PeriodicIO {
        // Inputs
        public double position_degrees = 0.0;
        public double velocity_rps = 0.0;
        public double current = 0.0;
        public double output_voltage = 0.0;

        // Outputs
        public double demand = 0;
        public long stopTime;
        public State state = State.LOADED;

        public double loadSpeed;
        public double unloadSpeed;
        public double extraLoadRotations;
        public double extraUnloadRotations;
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
                System.out.println("periodic - mBeamBreak: " + mBeamBreak.get());

                if (mBeamBreak.get()) {
                    mPeriodicIO.stopTime = System.currentTimeMillis() + (long)(1000 *
                        (mPeriodicIO.extraLoadRotations / mPeriodicIO.loadSpeed));
                    mPeriodicIO.state = State.LOADING_WITH_CORAL;
                }

                System.out.println("LOADING_NO_CORAL - stopTime: " + mPeriodicIO.stopTime);
                System.out.println("LOADING_NO_CORAL - extraLoadRotations: " + mPeriodicIO.extraLoadRotations);
                System.out.println("LOADING_NO_CORAL - loadSpeed: " + mPeriodicIO.loadSpeed);

                // continue without waiting for next periodic loop, if no wait time after the load
                if (mPeriodicIO.extraLoadRotations > 0) break;
            case LOADING_WITH_CORAL:
                System.out.println("LOADING_WITH_CORAL - mPeriodicIO.stopTime: " + mPeriodicIO.stopTime);
                System.out.println("LOADING_WITH_CORAL - System.currentTimeMillis(): " + System.currentTimeMillis());

                if (System.currentTimeMillis() >= mPeriodicIO.stopTime) {
                    setSetpoint(0);
                    mPeriodicIO.state = State.LOADED;
                }
                break;
            case UNLOADING_WITH_CORAL:
                System.out.println("UNLOADING_WITH_CORAL periodic - mBeamBreak: " + mBeamBreak.get());

                if (!mBeamBreak.get()) {
                    mPeriodicIO.stopTime = System.currentTimeMillis() + (long)(1000 *
                        (mPeriodicIO.extraUnloadRotations / mPeriodicIO.unloadSpeed));
                    mPeriodicIO.state = State.UNLOADING_NO_CORAL;
                }
                // continue without waiting for next periodic loop, if no wait time after the unload
                if (mPeriodicIO.extraUnloadRotations > 0) break;
            case UNLOADING_NO_CORAL:
                if (System.currentTimeMillis() >= mPeriodicIO.stopTime) {
                    setSetpoint(0);
                    mPeriodicIO.state = State.IDLE;
                }
                break;
        }

        mPeriodicIO.position_degrees = mMotor.getRotorPosition().getValue().in(Units.Degrees) / CoralConstants.gearRatio;
        mPeriodicIO.current = mMotor.getTorqueCurrent().getValue().in(Units.Amps);
        mPeriodicIO.output_voltage = mMotor.getMotorVoltage().getValue().in(Units.Volts);
        mPeriodicIO.velocity_rps = mMotor.getVelocity().getValue().in(Units.RotationsPerSecond) / CoralConstants.gearRatio;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Coral");
        builder.setSafeState(this::disable);
        builder.setActuator(true);

        builder.addDoubleProperty("Coral Angle (degrees)", () -> mPeriodicIO.position_degrees, null);
        builder.addDoubleProperty("Coral Motor Rotations", () -> mMotor.getRotorPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Coral Demand", () -> mPeriodicIO.demand, null);
        builder.addDoubleProperty("Coral Velocity rps", () -> mPeriodicIO.velocity_rps, null);
        builder.addDoubleProperty("Coral Volts", () -> mPeriodicIO.output_voltage, null);
        builder.addDoubleProperty("Coral Current", () -> mPeriodicIO.current, null);
        builder.addStringProperty("Coral State", () -> mPeriodicIO.state.toString(), null);
    }

    public void initTrimmer() {
        Trimmer trimmer = Trimmer.getInstance();
        trimmer.add(
            "Coral",
            "Load speed",
            () -> mPeriodicIO.loadSpeed,
            (up) -> {mPeriodicIO.loadSpeed = Trimmer.increment(mPeriodicIO.loadSpeed, 0.01, 0.2, up);}
        );
        trimmer.add(
            "Coral",
            "Unload speed",
            () -> mPeriodicIO.unloadSpeed,
            (up) -> {mPeriodicIO.unloadSpeed = Trimmer.increment(mPeriodicIO.unloadSpeed, 0.01, 0.2, up);}
        );
        trimmer.add(
            "Coral",
            "Extra load rot",
            () -> mPeriodicIO.extraLoadRotations,
            (up) -> {mPeriodicIO.extraLoadRotations = Math.max(0, mPeriodicIO.extraLoadRotations + (up ? 0.1 : -0.1));}
        );
        trimmer.add(
            "Coral",
            "Extra unload rot",
            () -> mPeriodicIO.extraUnloadRotations,
            (up) -> {mPeriodicIO.extraUnloadRotations = Math.max(0, mPeriodicIO.extraUnloadRotations + (up ? 0.1 : -0.1));}
        );
    }
}
