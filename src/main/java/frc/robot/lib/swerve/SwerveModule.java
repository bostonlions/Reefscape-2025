package frc.robot.lib.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.math.util.Units.degreesToRotations;
import static edu.wpi.first.math.util.Units.rotationsToDegrees;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Ports;

public final class SwerveModule extends SubsystemBase {
    private mPeriodicIO mPeriodicIO = new mPeriodicIO();

    private final int kModuleNumber;
    public final String name;
    private final double kAngleOffset;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    private ModuleState targetModuleState;

    private static final double wheelCircumference = SwerveConstants.wheelDiameter * Math.PI;

    public SwerveModule(int moduleNumber, Constants moduleConstants) {
        kModuleNumber = moduleNumber;
        name = moduleConstants.name;
        kAngleOffset = moduleConstants.angleOffset;

        angleEncoder = new CANcoder(moduleConstants.cancoderID, Ports.CANBUS_DRIVE);
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID, Ports.CANBUS_DRIVE);
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID, Ports.CANBUS_DRIVE);

        setConfigs();
        mAngleMotor.setPosition(0);
        mDriveMotor.setPosition(0.0);
        resetToAbsolute();
    }

    private void setConfigs() {
        angleEncoder.getConfigurator().apply(SwerveConstants.cancoderConfig);
        mAngleMotor.getConfigurator().apply(SwerveConstants.angleConfig);
        mDriveMotor.getConfigurator().apply(SwerveConstants.driveConfig);
    }

    public static final class Constants {
        public final int driveMotorID;
        public final int angleMotorID;
        public final int cancoderID;
        public final double angleOffset;
        public final String name;

        public Constants(int driveMotorID, int angleMotorID, int canCoderID, double angleOffset, String name) {
            this.driveMotorID = driveMotorID;
            this.angleMotorID = angleMotorID;
            this.cancoderID = canCoderID;
            this.angleOffset = angleOffset;
            this.name = name;
        }
    }

    public void setDesiredState(ModuleState desiredState, boolean isOpenLoop) {
        targetModuleState = desiredState;

        double targetAngle = targetModuleState.angle.getDegrees();

        if (isOpenLoop) mPeriodicIO.targetVelocity = targetModuleState.speedMetersPerSecond;
        else mPeriodicIO.targetVelocity = MathUtil.clamp(
            targetModuleState.speedMetersPerSecond,
            -SwerveConstants.maxAttainableSpeed,
            SwerveConstants.maxAttainableSpeed
        );

        // TODO - should this be ModuleState.optimize?
        if (shouldReverse(targetAngle, mPeriodicIO.rotationPosition)) {
            mPeriodicIO.targetVelocity = -mPeriodicIO.targetVelocity;
            targetAngle += 180.0;
        }

        targetAngle = MathUtil.inputModulus(targetAngle, mPeriodicIO.rotationPosition - 180,
            mPeriodicIO.rotationPosition + 180);

        mPeriodicIO.rotationDemand = degreesToRotations(targetAngle) *
            SwerveConstants.angleGearRatio; //this is a duplicate

        if (isOpenLoop) {
            mPeriodicIO.driveControlMode = ControlModeState.PercentOutput;
            mPeriodicIO.driveDemand = mPeriodicIO.targetVelocity / SwerveConstants.maxSpeed;
        } else {
            mPeriodicIO.driveControlMode = ControlModeState.Velocity;
            mPeriodicIO.driveDemand = (mPeriodicIO.targetVelocity / wheelCircumference) *
                SwerveConstants.driveGearRatio;
        }
    }

    private void resetToAbsolute() {
        double angle = MathUtil.inputModulus(getCanCoder() - kAngleOffset,
            mPeriodicIO.rotationPosition - 180, mPeriodicIO.rotationPosition + 180);
        double absolutePosition = degreesToRotations(angle) * SwerveConstants.angleGearRatio;
        mAngleMotor.setPosition(absolutePosition);
    }

    /** degrees */
    private double getCanCoder() {
        return MathUtil.inputModulus(angleEncoder.getAbsolutePosition().getValueAsDouble() * 360,
            0, 360);
    }

    public ModuleState getState() {
        return new ModuleState(mPeriodicIO.drivePosition, edu.wpi.first.math.geometry.Rotation2d.
            fromDegrees(mPeriodicIO.rotationPosition), mPeriodicIO.velocity);
    }

    @Override
    public void periodic() {
        /* read periodic inputs */

        mPeriodicIO.timestamp = Timer.getFPGATimestamp();

        mPeriodicIO.velocity = mDriveMotor.getRotorVelocity().getValue().in(Units.RotationsPerSecond) *
            wheelCircumference / SwerveConstants.driveGearRatio;

        mPeriodicIO.rotationPosition = mAngleMotor.getRotorPosition().getValue().in(Units.Degrees) /
            SwerveConstants.angleGearRatio;

        mPeriodicIO.drivePosition = mDriveMotor.getRotorPosition().getValue().in(Units.Rotations) *
            wheelCircumference / SwerveConstants.driveGearRatio;

        /* write periodic outputs */

        double targetAngle = targetModuleState.angle.getDegrees();
        if (shouldReverse(targetAngle, mPeriodicIO.rotationPosition)) {
            mPeriodicIO.targetVelocity = -mPeriodicIO.targetVelocity;
            targetAngle += 180.0;
        }
        targetAngle = MathUtil.inputModulus(targetAngle, mPeriodicIO.rotationPosition - 180,
            mPeriodicIO.rotationPosition + 180);

        mPeriodicIO.rotationDemand = degreesToRotations(targetAngle) * SwerveConstants.angleGearRatio;

        mAngleMotor.setControl(new PositionDutyCycle(mPeriodicIO.rotationDemand));

        if (mPeriodicIO.driveControlMode == ControlModeState.Velocity) mDriveMotor.setControl(
            new VelocityTorqueCurrentFOC(mPeriodicIO.driveDemand)
        ); else mDriveMotor.setControl(new DutyCycleOut(mPeriodicIO.driveDemand)); // TODO: previously this had extra args: true, false, false, false - but I can't find a constructor with that signature
    }

    private static final class mPeriodicIO {
        // Inputs
        public double timestamp = 0.0;
        public double targetVelocity = 0.0;
        public double rotationPosition = 0.0;
        public double drivePosition = 0.0;
        public double velocity = 0.0;

        // Outputs
        public ControlModeState driveControlMode = ControlModeState.PercentOutput;
        public double rotationDemand;
        public double driveDemand;
    }

    private enum ControlModeState{ PercentOutput, Velocity }

    public int moduleNumber() {
        return kModuleNumber;
    }

    public void setDriveNeutralBrake(boolean wantBrake) {
        if (wantBrake) {
            mDriveMotor.setNeutralMode(NeutralModeValue.Brake);
            // TODO does this also want to be Brake?
            mAngleMotor.setNeutralMode(NeutralModeValue.Coast);
        } else {
            mDriveMotor.setNeutralMode(NeutralModeValue.Coast);
            // Brake angle motors when coasting drive
            mAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        }
    }

    private boolean shouldReverse(double goalAngle, double currentAngle) {
        double diff = (goalAngle - currentAngle + 720.) % 360.;
        return diff > 90. && diff < 270.;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Angle Position", () -> mPeriodicIO.rotationPosition, null);
        builder.addDoubleProperty("CANCODER Position", () -> getCanCoder(), null);
        builder.addDoubleProperty("Drive Position", () -> mPeriodicIO.drivePosition, null);
        builder.addDoubleProperty("Velocity", () -> mPeriodicIO.velocity, null);
        builder.addDoubleProperty("Target Angle", () -> rotationsToDegrees(mPeriodicIO.rotationDemand) / SwerveConstants.angleGearRatio, null);
        builder.addDoubleProperty("Target Velocity", () -> mPeriodicIO.targetVelocity, null);
        builder.addStringProperty("Control Mode", () -> mPeriodicIO.driveControlMode.toString(), null);
    }
}
