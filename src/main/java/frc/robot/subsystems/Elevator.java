package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.Position;
import static frc.robot.Constants.ElevatorConstants.heights;
import static frc.robot.Constants.ElevatorConstants.positionOrder;
import frc.robot.Ports;
import frc.robot.lib.Util.Conversions;

public class Elevator extends SubsystemBase {

    public PeriodicIO mPeriodicIO = new PeriodicIO();

    public static Elevator mInstance;
    private final TalonFX mMain;
    private final TalonFX mFollower;

    public static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator();
        }
        return mInstance;
    }

    private Elevator() {
        mMain = new TalonFX(Ports.ELEVATOR_B, Ports.CANBUS_OPS);
        mMain.getConfigurator().apply(ElevatorConstants.motorConfig);

        mFollower = new TalonFX(Ports.ELEVATOR_A, Ports.CANBUS_OPS);
        mFollower.getConfigurator().apply(ElevatorConstants.motorConfig);
        mFollower.setControl(new Follower(Ports.ELEVATOR_B, true));

        setNeutralBrake(false);
    }

    public void setNeutralBrake(boolean brake) {
        NeutralModeValue wantedMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        mMain.setNeutralMode(wantedMode);
        mFollower.setNeutralMode(wantedMode);
    }

    public void zeroWhenDisabled() {
        if (mPeriodicIO.position < 0.0) {
            markMin();
            setTarget(Position.STOW);
        }
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

    public void markMin() {
        mMain.setPosition(metersToRotations(heights.get(Position.MIN)));
    }

    public void markMax() {
        mMain.setPosition(metersToRotations(heights.get(Position.MAX)));
    }

    public void setTarget(Position p) {
        mPeriodicIO.targetPosition = p;
        mPeriodicIO.targetHeight = p == Position.MANUAL ? mPeriodicIO.manualTargetHeight : heights.get(p);
        setNeutralBrake(true);
        setSetpointMotionMagic(mPeriodicIO.targetHeight);
    }

    public int getClosestStepNum(boolean goingUp) {
        Position p = mPeriodicIO.targetPosition;
        double h = mPeriodicIO.position;
        if (p == Position.MIN) { return -1; }
        if (p == Position.MAX) { return positionOrder.size(); }
        if (p != Position.MANUAL) {
            return positionOrder.indexOf(p);
        }
        if (goingUp) {
            for(int i = 0; i < positionOrder.size(); i++) {
                if (heights.get(positionOrder.get(i)) > h) return i - 1;
            }
            return positionOrder.size();
        }
        else {
            for(int i = positionOrder.size() - 1; i >= 0; i--) {
                if (heights.get(positionOrder.get(i)) < h) return i + 1;
            }
            return -1;
        }
    }

    public void stepUp() {
        int curStep = getClosestStepNum(true);
        if (curStep < positionOrder.size() - 1) {
            setTarget(positionOrder.get(curStep + 1));
        }
    }

    public void stepDown() {
        int curStep = getClosestStepNum(false);
        if (curStep > 0) {
            setTarget(positionOrder.get(curStep - 1));
        }
    }

    public static class PeriodicIO {
        // Inputs
        private double voltage;
        private double current;
        private double position;
        private double velocity;
        private double torqueCurrent;

        // Outputs
        private double demand;
        private Position targetPosition;
        private double targetHeight;
        private double manualTargetHeight = heights.get(Position.STOW);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Elevator Position Meters", () -> mPeriodicIO.position, null);
        builder.addDoubleProperty("Elevator Position Inches", () -> Conversions.metersToInches(mPeriodicIO.position), null);
        builder.addDoubleProperty("Elevator Motor Rotations", () -> metersToRotations(mPeriodicIO.position), null);
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
    }

    @Override
    public void periodic() {
        mPeriodicIO.voltage = mMain.getMotorVoltage().getValue().in(Units.Volts);
        mPeriodicIO.current = mMain.getStatorCurrent().getValue().in(Units.Amps);
        mPeriodicIO.position = rotationsToMeters(mMain.getRotorPosition().getValue().in(Units.Rotations));
        mPeriodicIO.velocity = mMain.getRotorVelocity().getValue().in(Units.RotationsPerSecond);
        mPeriodicIO.torqueCurrent = mMain.getTorqueCurrent().getValueAsDouble();

        // have we hit the top or bottom?
        if ((mPeriodicIO.torqueCurrent > ElevatorConstants.limitTorque) &&
            mPeriodicIO.velocity > -ElevatorConstants.limitVelocity
        ) {
            markMin();
            setTarget(Position.STOW);
        }
        else if ((mPeriodicIO.torqueCurrent < -ElevatorConstants.limitTorque) &&
            mPeriodicIO.velocity < ElevatorConstants.limitVelocity
        ) {
            markMax();
            setTarget(Position.BARGE);
        }
    }
}
