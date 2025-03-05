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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

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
        markPosition(Position.STOW);
        initTrimmer();
    }

    /* Commands */
    public Command nudgeUpCommand() { return new InstantCommand(() -> nudge(-1)); }
    public Command nudgeDownCommand() { return new InstantCommand(() -> nudge(1)); }
    public Command nudgeStopCommand() { return new InstantCommand(() -> nudge(0)); }
    public Command gotoCommand(Position p) { return new InstantCommand(() -> setTarget(p)); }
    public Command extendCommand() { return gotoCommand(Position.LATCH); }
    public Command climbCommand() {
        return gotoCommand(Position.DROP)
            .andThen(new WaitUntilCommand(() -> !mPeriodicIO.moving))
            .andThen(new WaitCommand(ClimberHookConstants.climbDelay))
            .andThen(gotoCommand(Position.CLIMBED));
    }

    /** Only used in footReleaseCommand (not a real class variable) */
    private double startingExtension;

    public Command footReleaseCommand() {
        return new InstantCommand(() -> {startingExtension = mPeriodicIO.extension; setSetpointMotionMagic(startingExtension + ClimberHookConstants.footReleaseRotations);})
            .andThen(new WaitCommand(ClimberHookConstants.footReleaseDelay))
            .andThen(new InstantCommand(() -> setSetpointMotionMagic(startingExtension)));
    }

    private void setConfig() {
        mMotor.getConfigurator().apply(motorConfig);
    }

    public void markPosition(Position p) {
        mMotor.setPosition(extensions.get(p) * ClimberHookConstants.gearRatio);
    }

    public void setWantNeutralBrake(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mMotor.setNeutralMode(mode);
    }

    public void setSetpointMotionMagic(double winchRotations) {
        mPeriodicIO.demand = winchRotations * ClimberHookConstants.gearRatio;
        mMotor.setControl(new MotionMagicDutyCycle(mPeriodicIO.demand));
    }

    public void setTarget(Position p) {
        mPeriodicIO.targetExtension = p == Position.MANUAL ? mPeriodicIO.manualTargetExtension : extensions.get(p);
        ClimberHookConstants.motorConfig.MotionMagic.MotionMagicCruiseVelocity = (
            (p == Position.CLIMBED ? ClimberHookConstants.climbSpeed : ClimberHookConstants.extendSpeed) * ClimberHookConstants.gearRatio
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
            // mPeriodicIO.demand = mMotor.getRotorPosition().getValueAsDouble();
            // mMotor.setControl(new MotionMagicDutyCycle(mPeriodicIO.demand));
            mMotor.setControl(new MotionMagicVelocityDutyCycle(0));

        }
    }

    public double getTorqueCurrent(){
        return mPeriodicIO.current;
    }

    private static final class PeriodicIO {
        /* INPUTS */
    
        /** In winch rotations */
        private double extension;
        private double current;
        private double output_voltage;
        private double velocity;
        private boolean moving;

        // Outputs
        private double targetExtension;
        private double demand;
        private double manualTargetExtension = extensions.get(Position.STOW);
        private double nudgeSpeed = ClimberHookConstants.nudgeSpeed;
    }

    @Override
    public void periodic() {
        mPeriodicIO.extension = mMotor.getRotorPosition().getValueAsDouble() / ClimberHookConstants.gearRatio;
        mPeriodicIO.current = mMotor.getTorqueCurrent().getValue().in(Units.Amps);
        mPeriodicIO.output_voltage = mMotor.getMotorVoltage().getValue().in(Units.Volts);
        mPeriodicIO.velocity = mMotor.getVelocity().getValue().in(Units.RotationsPerSecond);

        // Have we hit the min or max?
        if ((mPeriodicIO.current < -ClimberHookConstants.limitTorque) &&
            mPeriodicIO.velocity > -ClimberHookConstants.limitVelocity
        ) {
            markPosition(Position.MIN);
            setTarget(Position.CLIMBED);
        } else if ((mPeriodicIO.current > ClimberHookConstants.limitTorque) &&
            mPeriodicIO.velocity < ClimberHookConstants.limitVelocity
        ) {
            markPosition(Position.MAX);
            setTarget(Position.DROP);
        }

        // Have we finished moving?
        if (mPeriodicIO.moving && MathUtil.isNear(mPeriodicIO.extension, mPeriodicIO.targetExtension,
            ClimberHookConstants.extensionTolerance)) mPeriodicIO.moving = false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("ClimberHook");
        builder.setActuator(true);

        builder.addDoubleProperty("Winch Rotations", () -> mPeriodicIO.extension, null);
        builder.addDoubleProperty("Motor Rotations", () -> mMotor.getRotorPosition().getValueAsDouble(), null);
        builder.addDoubleProperty("Demand Winch Rot", () -> mPeriodicIO.demand, null);
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
