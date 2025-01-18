package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Ports;
import frc.robot.lib.loops.ILooper;
import frc.robot.lib.loops.Loop;
import frc.robot.lib.Util.Conversions;
import frc.robot.Constants.CoralConstants;

public class Coral extends Subsystem {

    private static Coral mInstance;
    private TalonFX mMotor;

    private mPeriodicIO mPeriodicIO = new mPeriodicIO();

    public static Coral getInstance() {
        if (mInstance == null) {
            mInstance = new Coral();
        }
        return mInstance;
    }

    private Coral() {
        mMotor = new TalonFX(Ports.CORAL_DRIVE, Ports.CANBUS_UPPER);
        // configs from constants
        mMotor.getConfigurator().apply(CoralConstants.motorConfig());

        setWantNeutralBrake(true);
        mMotor.setPosition(0);
    }

    public void resetToAbsolute() {
        // mMotor.setPosition(0);
    }

    public void setWantNeutralBrake(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mMotor.setNeutralMode(mode);
    }

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                // resetToAbsolute();

            }

            @Override
            public void onLoop(double timestamp) {
                if (mPeriodicIO.position_degrees<0){
                    mMotor.setPosition(0);
                }

            }

            @Override
            public void onStop(double timestamp) {
                setWantNeutralBrake(true);
            }
        });
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mPeriodicIO.mControlModeState == ControlModeState.MOTION_MAGIC) {

            // mMotor.setControl(new MotionMagicDutyCycle(mPeriodicIO.demand, true, 0, 0, false, false, false));
            mMotor.setControl(new MotionMagicDutyCycle(mPeriodicIO.demand).withEnableFOC(true));
        } else if (mPeriodicIO.mControlModeState == ControlModeState.OPEN_LOOP) {
            mMotor.setControl(new DutyCycleOut(mPeriodicIO.demand));
        }
    }

    public void setSetpointMotionMagic(double degrees) {
        if (mPeriodicIO.mControlModeState != ControlModeState.MOTION_MAGIC) {
            mPeriodicIO.mControlModeState = ControlModeState.MOTION_MAGIC;
        }

        double rotationDemand = Conversions.degreesToRotation(degrees, CoralConstants.kGearRatio);
        mPeriodicIO.demand = rotationDemand;
    }

    public void setDemandOpenLoop(double demand) {
        if (mPeriodicIO.mControlModeState != ControlModeState.OPEN_LOOP) {
            mPeriodicIO.mControlModeState = ControlModeState.OPEN_LOOP;
        }
        mPeriodicIO.demand = demand;
    }

    public double getAngleDeg() {
        return mPeriodicIO.position_degrees;
    }

    public double getTorqueCurrent(){
        return mPeriodicIO.current;
    }

    public static class mPeriodicIO {
        // Inputs
        public double timestamp = 0.0;
        public double targetVelocity = 0.0;
        public double position_degrees = 0.0;
        public double velocity_rps = 0.0;

        public double current = 0.0;
        public double output_voltage = 0.0;

        // Outputs
        public double demand = 0;
        public ControlModeState mControlModeState;
    }

    private enum ControlModeState {
        MOTION_MAGIC,
        OPEN_LOOP
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mPeriodicIO.position_degrees = mMotor.getRotorPosition().getValue().in(Units.Degrees) / CoralConstants.kGearRatio;
        mPeriodicIO.current = mMotor.getTorqueCurrent().getValue().in(Units.Amps);
        mPeriodicIO.output_voltage = mMotor.getMotorVoltage().getValue().in(Units.Volts);
        mPeriodicIO.velocity_rps = mMotor.getVelocity().getValue().in(Units.RotationsPerSecond) / CoralConstants.kGearRatio;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("ClimberHookAngle (degrees)", mPeriodicIO.position_degrees);
        SmartDashboard.putNumber("ClimberHook Motor Rotations", mMotor.getRotorPosition().getValueAsDouble());
        SmartDashboard.putNumber("ClimberHook Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("ClimberHook Velocity rad/s", mPeriodicIO.velocity_rps);
        SmartDashboard.putNumber("ClimberHook Demand", mPeriodicIO.demand);
        SmartDashboard.putNumber("ClimberHook Volts", mPeriodicIO.output_voltage);
        SmartDashboard.putNumber("ClimberHook Current", mPeriodicIO.current);
        // SmartDashboard.putString("ClimberHook Control State",
        // mPeriodicIO.mControlModeState.toString());
    }
}
