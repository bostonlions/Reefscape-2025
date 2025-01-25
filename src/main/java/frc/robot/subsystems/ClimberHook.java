package frc.robot.subsystems;

// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberHookConstants;
import frc.robot.Ports;
// import frc.robot.lib.loops.ILooper;
// import frc.robot.lib.loops.Loop;
import frc.robot.lib.Util.Conversions;

public class ClimberHook extends SubsystemBase {

    private static ClimberHook mInstance;
    private TalonFX mMotor;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    public static ClimberHook getInstance() {
        if (mInstance == null) {
            mInstance = new ClimberHook();
        }
        return mInstance;
    }

    private ClimberHook() {
        mMotor = new TalonFX(Ports.CLIMBER_HOOK_DRIVE, Ports.CANBUS_OPS);
        mMotor.getConfigurator().apply(ClimberHookConstants.motorConfig);

        setWantNeutralBrake(true);
        mMotor.setPosition(0);
    }

    public void resetToAbsolute() {
        // mMotor.setPosition(0);
    }

    public void disable() {
        // TODO
    }

    public void setWantNeutralBrake(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mMotor.setNeutralMode(mode);
    }

    // @Override
    // public void registerEnabledLoops(ILooper mEnabledLooper) {
    //     mEnabledLooper.register(new Loop() {
    //         @Override
    //         public void onStart(double timestamp) {
    //             // resetToAbsolute();

    //         }

    //         @Override
    //         public void onLoop(double timestamp) {
    //             if (mPeriodicIO.position_degrees<0){
    //                 mMotor.setPosition(0);
    //             }

    //         }

    //         @Override
    //         public void onStop(double timestamp) {
    //             setWantNeutralBrake(true);
    //         }
    //     });
    // }

    // @Override
    // public synchronized void writePeriodicOutputs() {
    //     if (mPeriodicIO.mControlModeState == ControlModeState.MOTION_MAGIC) {

    //         // mMotor.setControl(new MotionMagicDutyCycle(mPeriodicIO.demand, true, 0, 0, false, false, false));
    //         mMotor.setControl(new MotionMagicDutyCycle(mPeriodicIO.demand).withEnableFOC(true));
    //     } else if (mPeriodicIO.mControlModeState == ControlModeState.OPEN_LOOP) {
    //         mMotor.setControl(new DutyCycleOut(mPeriodicIO.demand));
    //     }
    // }

    public void setSetpointMotionMagic(double degrees) {
        if (mPeriodicIO.mControlModeState != ControlModeState.MOTION_MAGIC) {
            mPeriodicIO.mControlModeState = ControlModeState.MOTION_MAGIC;
        }

        double rotationDemand = Conversions.degreesToRotation(degrees, ClimberHookConstants.gearRatio);
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

    public static class PeriodicIO {
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
    public void periodic() {
        mPeriodicIO.position_degrees = mMotor.getRotorPosition().getValue().in(Units.Degrees) / ClimberHookConstants.gearRatio;
        mPeriodicIO.current = mMotor.getTorqueCurrent().getValue().in(Units.Amps);
        mPeriodicIO.output_voltage = mMotor.getMotorVoltage().getValue().in(Units.Volts);
        mPeriodicIO.velocity_rps = mMotor.getVelocity().getValue().in(Units.RotationsPerSecond) / ClimberHookConstants.gearRatio;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("ClimberHook Angle (degrees)", () -> mPeriodicIO.position_degrees, null);
        builder.addDoubleProperty("ClimberHook Motor Rotations", () -> mMotor.getRotorPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("ClimberHook Demand", () -> mPeriodicIO.demand, null);
        builder.addDoubleProperty("ClimberHook Velocity rad/s", () -> mPeriodicIO.velocity_rps, null);
        builder.addDoubleProperty("ClimberHook Demand", () -> mPeriodicIO.demand, null);
        builder.addDoubleProperty("ClimberHook Volts", () -> mPeriodicIO.output_voltage, null);
        builder.addDoubleProperty("ClimberHook Current", () -> mPeriodicIO.current, null);
        builder.setSafeState(this::disable);
        builder.setActuator(true);
    }

}
