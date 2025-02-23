package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.math.util.Units.metersToInches;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Ports;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.Position;

import static frc.robot.Constants.ElevatorConstants.heights;
import static frc.robot.Constants.ElevatorConstants.positionOrder;
import static frc.robot.Constants.ElevatorConstants.motorConfig;

public class Elevator extends SubsystemBase {
    public PeriodicIO mPeriodicIO = new PeriodicIO();
    private static Elevator mInstance;
    private final TalonFX mMain;
    private final TalonFX mFollower;

    public static Elevator getInstance() {
        if (mInstance == null) mInstance = new Elevator();
        return mInstance;
    }

    private Elevator() {
        mMain = new TalonFX(Ports.ELEVATOR_B, Ports.CANBUS_OPS);
        mFollower = new TalonFX(Ports.ELEVATOR_A, Ports.CANBUS_OPS);
        setMotorConfig(motorConfig);

        setNeutralBrake(false);
        mPeriodicIO.moving = false;

        initTrimmer();

        markMin(); // so elevator must start at min pos upon robot start

        // shouldn't need these but they fix our issue
        stepUp();
        stepDown();
    }

    /* Commands */
    public Command stepUpCommand() { return new InstantCommand(this::stepUp, this); }
    public Command stepDownCommand() { return new InstantCommand(this::stepDown, this); }
    public Command markMinCommand() { return new InstantCommand(this::markMin, this); }
    public Command forceDownCommand() { return new InstantCommand(this::forceDown, this); }
    public Command stepToCommand(Position p) {
        return new FunctionalCommand(
            () -> { System.out.println("Elevator stepped to " + p); setTarget(p); },
            () -> {},
            (b) -> {},
            () -> mPeriodicIO.targetPosition == p && doneMoving(),
            this
        );
    }

    private void setMotorConfig(TalonFXConfiguration config) {
        mMain.getConfigurator().apply(config);
        mFollower.getConfigurator().apply(config);
        mFollower.setControl(new Follower(Ports.ELEVATOR_B, true));
    }

    public void setNeutralBrake(boolean brake) {
        NeutralModeValue wantedMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mMain.setNeutralMode(wantedMode);
        mFollower.setNeutralMode(wantedMode);
    }

    private void disable() {
        setSetpointMotionMagic(mPeriodicIO.height);
        setNeutralBrake(false);
    }

    private static double metersToRotations(double distance) {
        return (distance / ElevatorConstants.wheelCircumference) * ElevatorConstants.gearRatio;
    }

    private static double rotationsToMeters(double rotations) {
        return rotations * ElevatorConstants.wheelCircumference / ElevatorConstants.gearRatio;
    }

    public void setSetpointMotionMagic(double distance) {
        System.out.println("Elevator setSetpointMotionMagic " + distance);
        mPeriodicIO.demand = metersToRotations(distance);
        mMain.setControl(new MotionMagicDutyCycle(mPeriodicIO.demand));
    }

    public void setTarget(Position p) {
        mPeriodicIO.targetPosition = p;
        mPeriodicIO.targetHeight = p == Position.MANUAL ? mPeriodicIO.manualTargetHeight : heights.get(p);
        mPeriodicIO.moving = true;
        setNeutralBrake(true);
        setSetpointMotionMagic(mPeriodicIO.targetHeight);
    }

    public int getClosestStepNum(boolean goingUp) {
        if (mPeriodicIO.targetPosition == Position.MIN) return -1;
        if (mPeriodicIO.targetPosition == Position.MAX) return positionOrder.size();
        if (mPeriodicIO.targetPosition == Position.MANUAL) {
            if (goingUp) {
                for(int i = 0; i < positionOrder.size(); i++)
                    if (heights.get(positionOrder.get(i)) > mPeriodicIO.manualTargetHeight) return i - 1;
                return positionOrder.size();
            }
            for(int i = positionOrder.size() - 1; i >= 0; i--)
                if (heights.get(positionOrder.get(i)) < mPeriodicIO.manualTargetHeight) return i + 1;
            return -1;
        }
        return positionOrder.indexOf(mPeriodicIO.targetPosition);
    }

    public boolean doneMoving() {
        return !mPeriodicIO.moving;
    }

    public void stepUp() {
        int curStep = getClosestStepNum(true);
        if (curStep < positionOrder.size() - 1) setTarget(positionOrder.get(curStep + 1));
    }

    public void stepDown() {
        int curStep = getClosestStepNum(false);
        if (curStep > 0) setTarget(positionOrder.get(curStep - 1));
    }

    public void markMin() {
        mPeriodicIO.targetPosition = Position.MIN;
        mPeriodicIO.targetHeight = heights.get(Position.MIN);
        mPeriodicIO.moving = false;

        mMain.setPosition(metersToRotations(heights.get(Position.MIN)));
    }

    /** Lower the elevator slowly until it recognizes that it's stuck at the bottom. */
    public void forceDown() {
        System.out.println("Elevator forceDown");
        mMain.setControl(new MotionMagicVelocityVoltage(-ElevatorConstants.resetSpeed));
    }

    private static final class PeriodicIO {
        // Inputs
        private double voltage;
        private double current;
        private double height;
        private double velocity;
        private double torqueCurrent;
        private boolean moving;

        // Outputs
        private double demand;
        private Position targetPosition = Position.MIN;
        private double targetHeight = 0.;
        private double manualTargetHeight = heights.get(positionOrder.get(0));
    }

    @Override
    public void periodic() {
        mPeriodicIO.voltage = mMain.getMotorVoltage().getValue().in(Units.Volts);
        mPeriodicIO.current = mMain.getStatorCurrent().getValue().in(Units.Amps);
        mPeriodicIO.height = rotationsToMeters(mMain.getRotorPosition().getValue().in(Units.Rotations));
        mPeriodicIO.velocity = mMain.getRotorVelocity().getValue().in(Units.RotationsPerSecond);
        mPeriodicIO.torqueCurrent = mMain.getTorqueCurrent().getValueAsDouble();

        /* Do we think we're below min? if so, do ... something? */
        if (mPeriodicIO.height < heights.get(Position.MIN)) {
            // forceDown();
        }

        /* Have we hit the top or bottom? */
        if ((mPeriodicIO.torqueCurrent < -ElevatorConstants.bottomLimitTorque) &&
            mPeriodicIO.velocity > -ElevatorConstants.limitVelocity
        ) {
            System.out.println("Elevator bottom limit hit, marking min height");
            markMin();
            setTarget(positionOrder.get(0));
        }
        else if ((mPeriodicIO.torqueCurrent > ElevatorConstants.topLimitTorque) &&
            mPeriodicIO.velocity < ElevatorConstants.limitVelocity
        ) {
            System.out.println("Elevator top limit hit, marking max height");
            mMain.setPosition(metersToRotations(heights.get(Position.MAX))); //mark max
            setTarget(positionOrder.get(positionOrder.size() - 1));
        }

        /* Have we finished moving? */
        if (mPeriodicIO.moving && MathUtil.isNear(mPeriodicIO.height, mPeriodicIO.targetHeight,
            ElevatorConstants.heightTolerance)) mPeriodicIO.moving = false;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator");
        builder.setSafeState(this::disable);
        builder.setActuator(true);

        builder.addDoubleProperty("Position Meters", () -> mPeriodicIO.height, null);
        builder.addDoubleProperty("Position Inches", () -> metersToInches(mPeriodicIO.height), null);
        builder.addDoubleProperty("Motor Rotations", () -> metersToRotations(mPeriodicIO.height), null);
        builder.addBooleanProperty("Moving", () -> mPeriodicIO.moving, null);
        builder.addDoubleProperty("Demand", () -> mPeriodicIO.demand, null);
        builder.addDoubleProperty("Velocity", () -> mPeriodicIO.velocity, null);
        builder.addDoubleProperty("Output Volts", () -> mPeriodicIO.voltage, null);
        builder.addDoubleProperty("Current", () -> mPeriodicIO.current, null);
        builder.addDoubleProperty("Torque Current", () -> mPeriodicIO.torqueCurrent, null);
        builder.addStringProperty("State", () -> mPeriodicIO.targetPosition.toString(), null);
    }

    private void initTrimmer() {
        Trimmer trimmer = Trimmer.getInstance();

        trimmer.add(
            "Elevator",
            "Up-Down 1cm",
            () -> mPeriodicIO.targetHeight,
            (up) -> {mPeriodicIO.manualTargetHeight = mPeriodicIO.targetHeight + (up ? 0.01 : -0.01); setTarget(Position.MANUAL);}
        );
        trimmer.add(
            "Elevator",
            "Speed",
            () -> motorConfig.MotionMagic.MotionMagicCruiseVelocity,
            (up) -> {motorConfig.MotionMagic.MotionMagicCruiseVelocity = Trimmer.increment(motorConfig.MotionMagic.MotionMagicCruiseVelocity, 1, 0.2, up); setMotorConfig(motorConfig);}
        );
        trimmer.add(
            "Elevator",
            "Accel",
            () -> motorConfig.MotionMagic.MotionMagicAcceleration,
            (up) -> {motorConfig.MotionMagic.MotionMagicAcceleration = Trimmer.increment(motorConfig.MotionMagic.MotionMagicAcceleration, 1, 0.2, up); setMotorConfig(motorConfig);}
        );
    }
}
