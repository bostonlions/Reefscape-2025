package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.Position;
import static frc.robot.Constants.ElevatorConstants.heights;
import static frc.robot.Constants.ElevatorConstants.positionOrder;
import frc.robot.Ports;
import frc.robot.lib.Util;
import frc.robot.lib.Util.Conversions;

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
        setMotorConfig(ElevatorConstants.motorConfig);

        setNeutralBrake(false);
        mPeriodicIO.moving = false;
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

    public void disable() {
        setSetpointMotionMagic(mPeriodicIO.height);
        setNeutralBrake(false);
    }

    public double metersToRotations(double distance) {
        return Conversions.metersToRotations(distance, ElevatorConstants.wheelCircumference, ElevatorConstants.gearRatio);
    }

    public double rotationsToMeters(double rotations) {
        return Conversions.rotationsToMeters(rotations, ElevatorConstants.wheelCircumference, ElevatorConstants.gearRatio);
    }

    public void setSetpointMotionMagic(double distance) {
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
            } //else:
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
        mMain.setPosition(metersToRotations(heights.get(Position.MIN)));
    }

    public static class PeriodicIO {
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
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Elevator Position Meters", () -> mPeriodicIO.height, null);
        builder.addDoubleProperty("Elevator Position Inches", () -> Conversions.metersToInches(mPeriodicIO.height), null);
        builder.addDoubleProperty("Elevator Motor Rotations", () -> metersToRotations(mPeriodicIO.height), null);
        builder.addBooleanProperty("Elevator Moving", () -> mPeriodicIO.moving, null);
        builder.addDoubleProperty("Elevator Demand", () -> mPeriodicIO.demand, null);
        builder.addDoubleProperty("Elevator Velocity", () -> mPeriodicIO.velocity, null);
        builder.addDoubleProperty("Elevator Output Volts", () -> mPeriodicIO.voltage, null);
        builder.addDoubleProperty("Elevator Current", () -> mPeriodicIO.current, null);
        builder.addDoubleProperty("Elevator Torque Current", () -> mPeriodicIO.torqueCurrent, null);
        builder.addBooleanProperty("Elevator Mark Min", () -> false, (v) -> {if(v) markMin();});
        builder.addBooleanProperty("Elevator Step UP", () -> false, (v) -> {if(v) stepUp();});
        builder.addBooleanProperty("Elevator Step DOWN", () -> false, (v) -> {if(v) stepDown();});
        builder.addDoubleProperty("Elevator Manual Target", () -> mPeriodicIO.manualTargetHeight, (v) -> {mPeriodicIO.manualTargetHeight = v;});
        builder.addBooleanProperty("Elevator Manual Go", () -> false, (v) -> {if(v) setTarget(Position.MANUAL);});
        builder.addStringProperty("Elevator State", () -> mPeriodicIO.targetPosition.toString(), null);
        builder.addDoubleProperty(
            "Speed limit rps",
            () -> ElevatorConstants.motorConfig.MotionMagic.MotionMagicCruiseVelocity,
            (v) -> {
                ElevatorConstants.motorConfig.MotionMagic.MotionMagicCruiseVelocity = v;
                setMotorConfig(ElevatorConstants.motorConfig);
            }
        );
        builder.addDoubleProperty(
            "Accel. limit rps2",
            () -> ElevatorConstants.motorConfig.MotionMagic.MotionMagicAcceleration,
            (v) -> {
                ElevatorConstants.motorConfig.MotionMagic.MotionMagicAcceleration = v;
                setMotorConfig(ElevatorConstants.motorConfig);
            });
        builder.setSafeState(this::disable);
        builder.setActuator(true);
    }

    @Override
    public void periodic() {
        mPeriodicIO.voltage = mMain.getMotorVoltage().getValue().in(Units.Volts);
        mPeriodicIO.current = mMain.getStatorCurrent().getValue().in(Units.Amps);
        mPeriodicIO.height = rotationsToMeters(mMain.getRotorPosition().getValue().in(Units.Rotations));
        mPeriodicIO.velocity = mMain.getRotorVelocity().getValue().in(Units.RotationsPerSecond);
        mPeriodicIO.torqueCurrent = mMain.getTorqueCurrent().getValueAsDouble();

        /* Have we hit the top or bottom? */
        if ((mPeriodicIO.torqueCurrent < -ElevatorConstants.limitTorque) &&
            mPeriodicIO.velocity > -ElevatorConstants.limitVelocity
        ) {
            markMin();
            setTarget(positionOrder.get(0));
        }
        else if ((mPeriodicIO.torqueCurrent > ElevatorConstants.limitTorque) &&
            mPeriodicIO.velocity < ElevatorConstants.limitVelocity
        ) {
            mMain.setPosition(metersToRotations(heights.get(Position.MAX))); //mark max
            setTarget(positionOrder.get(positionOrder.size() - 1));
        }

        /* Have we finished moving? */
        if (mPeriodicIO.moving &&
            Util.epsilonEquals(mPeriodicIO.height, mPeriodicIO.targetHeight, ElevatorConstants.heightTolerance)
        ) {
            mPeriodicIO.moving = false;
        }

        /* Dashboard actions */
        if (targetGo.getBoolean(false)) {
            mPeriodicIO.manualTargetHeight = Util.limit(targetEntry.getDouble(0), heights.get(Position.MIN), heights.get(Position.MAX));
            setTarget(Position.MANUAL);
            targetGo.setBoolean(false);
        }
    }

    private ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
    private GenericEntry targetEntry = tab.add("Manual Target", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .getEntry();
    private GenericEntry targetGo = tab.add("Manual Go", false).getEntry();
}
