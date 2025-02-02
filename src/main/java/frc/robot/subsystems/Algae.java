package frc.robot.subsystems;

import edu.wpi.first.units.Units;
// import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import frc.robot.Ports;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.AlgaeConstants.Position;
import static frc.robot.Constants.AlgaeConstants.angles;

public class Algae extends SubsystemBase {
    private static Algae mInstance;
    private TalonFX mDriveMotor;
    private TalonFX mAngleMotor;
    private DigitalInput mBeamBreak;
    private CANcoder mCANcoder;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private enum State {
        IDLE, GROUND_INTAKE_NO_ALGAE, GROUND_INTAKE_WITH_ALGAE, REEF_INTAKE_NO_ALGAE,
        REEF_INTAKE_WITH_ALGAE, LOADED_UP, LOADED_DOWN, UNLOADING_PROCESSOR_WITH_ALGAE,
        UNLOADING_PROCESSOR_NO_ALGAE, UNLOADING_BARGE_WITH_ALGAE, UNLOADING_BARGE_NO_ALGAE
    }
    public enum RequestState { DOWN, UP, RELEASE, NONE }

    public static Algae getInstance() {
        if (mInstance == null) mInstance = new Algae();
        return mInstance;
    }

    private Algae() {
        AlgaeConstants.angleMotorConfig.Feedback.FeedbackRemoteSensorID = mCANcoder.getDeviceID();
        AlgaeConstants.angleMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        mDriveMotor = new TalonFX(Ports.ALGAE_DRIVE, Ports.CANBUS_OPS);
        mDriveMotor.getConfigurator().apply(AlgaeConstants.driveMotorConfig);

        mAngleMotor = new TalonFX(Ports.ALGAE_ANGLE, Ports.CANBUS_OPS);
        mAngleMotor.getConfigurator().apply(AlgaeConstants.angleMotorConfig);

        mBeamBreak = new DigitalInput(Ports.ALGAE_BEAMBREAK);

        mCANcoder = new CANcoder(Ports.ALGAE_CANCODER, Ports.CANBUS_OPS);
        mCANcoder.getConfigurator().apply(AlgaeConstants.cancoderConfig);
    }

    public void disable() {
        // TODO
    }

    public void setAngleSetpoint(double angle) {
        mPeriodicIO.C_demand = (angle - AlgaeConstants.cancoderOffset)/360.;
        mAngleMotor.setControl(new MotionMagicDutyCycle(mPeriodicIO.C_demand));

        IMPORTANT// TODO: move drive to keep algae centered, handled in this method
    }

    public void setDriveSpeed(double rps) {
        mPeriodicIO.D_demand = rps * AlgaeConstants.gearRatio;
        mDriveMotor.setControl(new MotionMagicVelocityDutyCycle(mPeriodicIO.D_demand));
        IMPORTANT// TODO: decouple angle and drive motors
    }

    public void request(RequestState requestState) {
        if (requestState == RequestState.NONE) throw new IllegalArgumentException(
            "Cannot use algae request method with argument RequestState.NONE"
        );
        TODO
    }

    private final class PeriodicIO {
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
        public double C_demand = 0; public double D_demand = 0;
        public State state = State.IDLE;
        public RequestState requestState = RequestState.NONE;
        public long stopTime;
    }

    @Override
    public void periodic() {
        switch (mPeriodicIO.state) {
            case GROUND_INTAKE_NO_ALGAE:
                if (mBeamBreak.get()) {
                    mPeriodicIO.stopTime = System.currentTimeMillis() + (long)(1000 *
                        (AlgaeConstants.extraGroundIntakeRotations / AlgaeConstants.groundIntakeSpeed));
                    mPeriodicIO.state = State.GROUND_INTAKE_WITH_ALGAE;
                }
                if (AlgaeConstants.extraGroundIntakeRotations > 0) break; // we wanna check the next case
                // without waiting for next periodic loop, just if no wait time after the ground intake
            case GROUND_INTAKE_WITH_ALGAE:
                if (System.currentTimeMillis() >= mPeriodicIO.stopTime) {
                    setDriveSpeed(0);
                    setAngleSetpoint(angles.get(Position.PROCESSOR));
                    mPeriodicIO.state = State.LOADED_DOWN;
                } break;
            case REEF_INTAKE_NO_ALGAE:
                if (mBeamBreak.get()) {
                    mPeriodicIO.stopTime = System.currentTimeMillis() + (long)(1000 *
                        (AlgaeConstants.extraReefIntakeRotations / AlgaeConstants.reefIntakeSpeed));
                    mPeriodicIO.state = State.REEF_INTAKE_WITH_ALGAE;
                }
                if (AlgaeConstants.extraReefIntakeRotations > 0) break; // we wanna check the next case
                // without waiting for next periodic loop, just if no wait time after the ground intake
            case REEF_INTAKE_WITH_ALGAE:
                if (System.currentTimeMillis() >= mPeriodicIO.stopTime) {
                    setDriveSpeed(0);
                    setAngleSetpoint(angles.get(Position.BARGE));
                    mPeriodicIO.state = State.LOADED_UP;
                } break;
            case UNLOADING_PROCESSOR_WITH_ALGAE:
                if (!mBeamBreak.get()) {
                    mPeriodicIO.stopTime = System.currentTimeMillis() + (long)(1000 *
                        (AlgaeConstants.extraProcessorUnloadRotations / AlgaeConstants.processorUnloadSpeed));
                    mPeriodicIO.state = State.UNLOADING_PROCESSOR_NO_ALGAE;
                }
                if (AlgaeConstants.extraProcessorUnloadRotations > 0) break; // we wanna check the next case
                // without waiting for next periodic loop, just if no wait time after the ground intake
            case UNLOADING_PROCESSOR_NO_ALGAE:
                if (System.currentTimeMillis() >= mPeriodicIO.stopTime) {
                    setDriveSpeed(0);
                    setAngleSetpoint(angles.get(Position.STOW));
                    mPeriodicIO.state = State.IDLE;
                } break;
            case UNLOADING_BARGE_WITH_ALGAE:
                if (!mBeamBreak.get()) {
                    mPeriodicIO.stopTime = System.currentTimeMillis() + (long)(1000 *
                        (AlgaeConstants.extraBargeUnloadRotations / AlgaeConstants.bargeUnloadSpeed));
                    mPeriodicIO.state = State.UNLOADING_BARGE_NO_ALGAE;
                }
                if (AlgaeConstants.extraBargeUnloadRotations > 0) break; // we wanna check the next case
                // without waiting for next periodic loop, just if no wait time after the ground intake
            case UNLOADING_BARGE_NO_ALGAE:
                if (System.currentTimeMillis() >= mPeriodicIO.stopTime) {
                    setDriveSpeed(0);
                    setAngleSetpoint(angles.get(Position.STOW));
                    mPeriodicIO.state = State.IDLE;
                } break;
            default: break;
        }
        switch (mPeriodicIO.requestState) {
            case DOWN:
                if ((mPeriodicIO.state == State.IDLE) || (mPeriodicIO.state == State.REEF_INTAKE_NO_ALGAE)) {
                    setAngleSetpoint(angles.get(Position.GROUND_INTAKE));
                    mPeriodicIO.state = State.GROUND_INTAKE_NO_ALGAE;
                    mPeriodicIO.requestState = RequestState.NONE;
                } else if (mPeriodicIO.state == State.LOADED_UP) {
                    setAngleSetpoint(angles.get(Position.PROCESSOR));
                    mPeriodicIO.state = State.LOADED_DOWN;
                    mPeriodicIO.requestState = RequestState.NONE;
                } break;
            case UP:
                if ((mPeriodicIO.state == State.IDLE) || (mPeriodicIO.state == State.GROUND_INTAKE_NO_ALGAE)) {
                    setAngleSetpoint(angles.get(Position.REEF));
                    mPeriodicIO.state = State.REEF_INTAKE_NO_ALGAE;
                    mPeriodicIO.requestState = RequestState.NONE;
                } else if (mPeriodicIO.state == State.LOADED_DOWN) {
                    setAngleSetpoint(angles.get(Position.BARGE));
                    mPeriodicIO.state = State.LOADED_UP;
                    mPeriodicIO.requestState = RequestState.NONE;
                } break;
            case RELEASE:
                if (mPeriodicIO.state == State.LOADED_UP) {
                    setDriveSpeed(AlgaeConstants.bargeUnloadSpeed);
                    mPeriodicIO.state = State.UNLOADING_BARGE_WITH_ALGAE;
                    mPeriodicIO.requestState = RequestState.NONE;
                } else if (mPeriodicIO.state == State.LOADED_DOWN) {
                    setDriveSpeed(-AlgaeConstants.processorUnloadSpeed);
                    mPeriodicIO.state = State.UNLOADING_PROCESSOR_WITH_ALGAE;
                    mPeriodicIO.requestState = RequestState.NONE;
                }
            case NONE: break;
        }

        mPeriodicIO.C_position_degrees = mAngleMotor.getRotorPosition().getValue().in(Units.Degrees) / AlgaeConstants.gearRatio;
        mPeriodicIO.C_current = mAngleMotor.getTorqueCurrent().getValue().in(Units.Amps);
        mPeriodicIO.C_output_voltage = mAngleMotor.getMotorVoltage().getValue().in(Units.Volts);
        mPeriodicIO.C_velocity_rps = mAngleMotor.getVelocity().getValue().in(Units.RotationsPerSecond) / AlgaeConstants.gearRatio;

        mPeriodicIO.D_position_degrees = mDriveMotor.getRotorPosition().getValue().in(Units.Degrees) / AlgaeConstants.gearRatio;
        mPeriodicIO.D_current = mDriveMotor.getTorqueCurrent().getValue().in(Units.Amps);
        mPeriodicIO.D_output_voltage = mDriveMotor.getMotorVoltage().getValue().in(Units.Volts);
        mPeriodicIO.D_velocity_rps = mDriveMotor.getVelocity().getValue().in(Units.RotationsPerSecond) / AlgaeConstants.gearRatio;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        // TODO: builder.addDoubleProperty(.....) etc
        builder.setSafeState(this::disable);
        builder.setActuator(true);
    }
}
