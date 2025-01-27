package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberHookConstants;
import frc.robot.Constants.ClimberHookConstants.Position;
import static frc.robot.Constants.ClimberHookConstants.extensions;
import frc.robot.Ports;
import frc.robot.lib.Util;
import frc.robot.lib.Util.Conversions;

public class ClimberHook extends SubsystemBase {
    private static ClimberHook mInstance;
    private TalonFX mMotor;
    private PeriodicIO mPeriodicIO = new PeriodicIO();

    public static ClimberHook getInstance() {
        if (mInstance == null) mInstance = new ClimberHook();
        return mInstance;
    }

    private ClimberHook() {
        mMotor = new TalonFX(Ports.CLIMBER_HOOK_DRIVE, Ports.CANBUS_OPS);
        mMotor.getConfigurator().apply(ClimberHookConstants.motorConfig);

        setWantNeutralBrake(true);
        mMotor.setPosition(0);
    }

    public void disable() {
        // TODO
    }

    public void setWantNeutralBrake(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mMotor.setNeutralMode(mode);
    }

    public void setSetpointMotionMagic(double degrees) {
        double rotationDemand = Conversions.degreesToRotation(degrees, ClimberHookConstants.gearRatio);
        mPeriodicIO.demand = rotationDemand;
    }

    public void setTarget(Position p) {
        mPeriodicIO.targetExtension = p == Position.MANUAL ? mPeriodicIO.manualTargetExtension : extensions.get(p);
        setWantNeutralBrake(true);
        setSetpointMotionMagic(mPeriodicIO.targetExtension);
    }

    public void toggleTarget() {
        double midpoint = (extensions.get(Position.IN) + extensions.get(Position.OUT)) / 2;
        if (mPeriodicIO.extension > midpoint) setTarget(Position.IN); else setTarget(Position.OUT);
    }

    public double getAngleDeg() {
        return mPeriodicIO.extension;
    }

    public double getTorqueCurrent(){
        return mPeriodicIO.current;
    }

    public static class PeriodicIO {
        // Inputs
        private double extension;
        private double current;
        private double output_voltage;
        private double velocity;
        private boolean moving;

        // Outputs
        private double targetExtension;
        private double demand;
        private double manualTargetExtension = extensions.get(Position.IN);
    }

    @Override
    public void periodic() {
        mPeriodicIO.extension = mMotor.getRotorPosition().getValue().in(Units.Degrees) /
            ClimberHookConstants.gearRatio;
        mPeriodicIO.current = mMotor.getTorqueCurrent().getValue().in(Units.Amps);
        mPeriodicIO.output_voltage = mMotor.getMotorVoltage().getValue().in(Units.Volts);
        mPeriodicIO.velocity = mMotor.getVelocity().getValue().in(Units.RotationsPerSecond) /
            ClimberHookConstants.gearRatio;

        // Have we hit the top or bottom?
        if ((mPeriodicIO.current < -ClimberHookConstants.limitTorque) &&
            mPeriodicIO.velocity > -ClimberHookConstants.limitVelocity
        ) {
            mMotor.setPosition(Conversions.degreesToRotation(extensions.get(Position.MIN),
                ClimberHookConstants.gearRatio)); //mark min
            setTarget(Position.IN);
        } else if ((mPeriodicIO.current > ClimberHookConstants.limitTorque) &&
            mPeriodicIO.velocity < ClimberHookConstants.limitVelocity
        ) {
            mMotor.setPosition(Conversions.degreesToRotation(extensions.get(Position.MAX),
                ClimberHookConstants.gearRatio)); //mark max
            setTarget(Position.OUT);
        }

        // Have we finished moving?
        if (mPeriodicIO.moving &&
            Util.epsilonEquals(mPeriodicIO.extension, mPeriodicIO.targetExtension, ClimberHookConstants.extensionTolerance)
        )
            mPeriodicIO.moving = false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("ClimberHook Angle (degrees)", () -> mPeriodicIO.extension, null);
        builder.addDoubleProperty("ClimberHook Motor Rotations", () -> mMotor.getRotorPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("ClimberHook Demand", () -> mPeriodicIO.demand, null);
        builder.addDoubleProperty("ClimberHook Velocity rad/s", () -> mPeriodicIO.velocity, null);
        builder.addDoubleProperty("ClimberHook Volts", () -> mPeriodicIO.output_voltage, null);
        builder.addDoubleProperty("ClimberHook Current", () -> mPeriodicIO.current, null);
        builder.addBooleanProperty("Climberhook Toggle", () -> false, (v) -> {if(v) toggleTarget();});
        builder.addDoubleProperty("ClimberHook Manual Target", () -> mPeriodicIO.manualTargetExtension, (v) -> {mPeriodicIO.manualTargetExtension = v;});
        builder.addBooleanProperty("ClimberHook Manual Go", () -> false, (v) -> {if(v) setTarget(Position.MANUAL);});
        builder.setSafeState(this::disable);
        builder.setActuator(true);
    }
}
