package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.math.util.Units.degreesToRotations;

import frc.robot.Constants.ClimberHookConstants;
import frc.robot.Constants.ClimberHookConstants.Position;
import frc.robot.Ports;

import static frc.robot.Constants.ClimberHookConstants.extensions;

public class ClimberHook extends SubsystemBase {
    private static ClimberHook mInstance;
    private TalonFX mMotor;
    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private TalonFXConfiguration motorConfig;

    public static ClimberHook getInstance() {
        if (mInstance == null) mInstance = new ClimberHook();
        return mInstance;
    }

    private ClimberHook() {
        mMotor = new TalonFX(Ports.CLIMBER_HOOK_DRIVE, Ports.CANBUS_OPS);
        motorConfig = ClimberHookConstants.motorConfig;
        setConfig();

        setWantNeutralBrake(true);
        setZero();
        initTrimmer();
    }

    /* Commands */
    public Command toggleCommand() { return new InstantCommand(this::toggleTarget, this); }
    public Command nudgeUpCommand() { return new InstantCommand(() -> this.nudge(-1)); }
    public Command nudgeDownCommand() { return new InstantCommand(() -> this.nudge(1)); }
    public Command nudgeStopCommand() { return new InstantCommand(() -> this.nudge(0)); }

    private void setConfig() {
        mMotor.getConfigurator().apply(motorConfig);
    }

    private void disable() {
        // TODO
    }

    public void setZero() {
        mMotor.setPosition(0);
    }

    public void setWantNeutralBrake(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mMotor.setNeutralMode(mode);
    }

    public void setSetpointMotionMagic(double degrees) {
        mPeriodicIO.demand = degreesToRotations(degrees) * ClimberHookConstants.gearRatio;
        mMotor.setControl(new MotionMagicDutyCycle(mPeriodicIO.demand));
    }

    public void setTarget(Position p) {
        mPeriodicIO.targetExtension = p == Position.MANUAL ? mPeriodicIO.manualTargetExtension : extensions.get(p);
        ClimberHookConstants.motorConfig.MotionMagic.MotionMagicCruiseVelocity = (
            p == Position.OUT ? ClimberHookConstants.fastSpeed : ClimberHookConstants.slowSpeed
        );
        setWantNeutralBrake(true);
        setSetpointMotionMagic(mPeriodicIO.targetExtension);
    }

    /**
     * Use xbox left joystick up/down to manually move in/out.
     * Edit the nudge speed with the trimmer.
     */
    public void nudge(int direction) {
        double speed = direction * mPeriodicIO.nudgeSpeed * ClimberHookConstants.gearRatio;
        if (direction != 0) {
            mMotor.setControl(new MotionMagicVelocityDutyCycle(speed));
        } else {
            mMotor.setPosition(0.);
            mMotor.setControl(new MotionMagicDutyCycle(0.));
        }
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

    private static final class PeriodicIO {
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
        private double nudgeSpeed = ClimberHookConstants.nudgeSpeed;
    }

    @Override
    public void periodic() {
        mPeriodicIO.extension = mMotor.getRotorPosition().getValue().in(Units.Degrees) / ClimberHookConstants.gearRatio;
        mPeriodicIO.current = mMotor.getTorqueCurrent().getValue().in(Units.Amps);
        mPeriodicIO.output_voltage = mMotor.getMotorVoltage().getValue().in(Units.Volts);
        mPeriodicIO.velocity = mMotor.getVelocity().getValue().in(Units.RotationsPerSecond) / ClimberHookConstants.gearRatio;

        // Have we hit the top or bottom?
        if ((mPeriodicIO.current < -ClimberHookConstants.limitTorque) &&
            mPeriodicIO.velocity > -ClimberHookConstants.limitVelocity
        ) {
            mMotor.setPosition(degreesToRotations(extensions.get(Position.MIN)) *
                ClimberHookConstants.gearRatio); //mark min
            setTarget(Position.IN);
        } else if ((mPeriodicIO.current > ClimberHookConstants.limitTorque) &&
            mPeriodicIO.velocity < ClimberHookConstants.limitVelocity
        ) {
            mMotor.setPosition(degreesToRotations(extensions.get(Position.MAX)) *
                ClimberHookConstants.gearRatio); //mark max
            setTarget(Position.OUT);
        }

        // Have we finished moving?
        if (mPeriodicIO.moving && MathUtil.isNear(mPeriodicIO.extension, mPeriodicIO.targetExtension,
            ClimberHookConstants.extensionTolerance)) mPeriodicIO.moving = false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ClimberHook");
        builder.setSafeState(this::disable);
        builder.setActuator(true);

        builder.addDoubleProperty("Angle (degrees)", () -> mPeriodicIO.extension, null);
        builder.addDoubleProperty("Motor Rotations", () -> mMotor.getRotorPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Demand", () -> mPeriodicIO.demand, null);
        builder.addDoubleProperty("Velocity rad_s", () -> mPeriodicIO.velocity, null);
        builder.addDoubleProperty("Volts", () -> mPeriodicIO.output_voltage, null);
        builder.addDoubleProperty("Current", () -> mPeriodicIO.current, null);
    }

    private void initTrimmer() {
        Trimmer trimmer = Trimmer.getInstance();

        trimmer.add(
            "Climberhook",
            "Nudge speed",
            () -> mPeriodicIO.nudgeSpeed,
            (up) -> {mPeriodicIO.nudgeSpeed = Trimmer.increment(mPeriodicIO.nudgeSpeed, 0.01, 0.2, up);}
        );
        trimmer.add(
            "Climberhook",
            "kP",
            () -> motorConfig.Slot0.kP,
            (up) -> {motorConfig.Slot0.kP = Trimmer.increment(motorConfig.Slot0.kP, 0.01, 0.2, up); setConfig();}
        );
        trimmer.add(
            "Climberhook",
            "kI",
            () -> motorConfig.Slot0.kI,
            (up) -> {motorConfig.Slot0.kI = Trimmer.increment(motorConfig.Slot0.kI, 0.01, 0.2, up); setConfig();}
        );
        trimmer.add(
            "Climberhook",
            "kD",
            () -> motorConfig.Slot0.kD,
            (up) -> {motorConfig.Slot0.kD = Trimmer.increment(motorConfig.Slot0.kD, 0.01, 0.2, up); setConfig();}
        );
    }
}
