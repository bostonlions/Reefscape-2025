package frc.robot.lib.drivers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.Constants;
import frc.robot.Ports;

public class Pigeon {
    private static Pigeon mInstance;

    public static Pigeon getInstance() {
        if (mInstance == null) mInstance = new Pigeon(Ports.PIGEON);
        return mInstance;
    }

    // Actual pigeon object
    private final Pigeon2 mGyro;

    // Configs
    private boolean inverted = Constants.SwerveConstants.invertGyro;
    private Rotation2d yawAdjustmentAngle = new Rotation2d();
    private Rotation2d rollAdjustmentAngle = new Rotation2d();
    private Rotation2d pitchAdjustmentAngle = new Rotation2d();

    private Pigeon(int port) {
        mGyro = new Pigeon2(port, Ports.CANBUS_DRIVE);
        mGyro.getConfigurator().apply(new Pigeon2Configuration());
    }

    public Rotation2d getYaw() {
        Rotation2d angle = getUnadjustedYaw().rotateBy(yawAdjustmentAngle.unaryMinus());
        if (inverted) return angle.unaryMinus();
        return Rotation2d.fromDegrees(angle.getDegrees()); //not 0-360?
    }

    public Rotation2d getRoll() {
        return getUnadjustedRoll().rotateBy(rollAdjustmentAngle.unaryMinus());
    }

    public Rotation2d getPitch() {
        return getUnadjustedPitch().rotateBy(pitchAdjustmentAngle.unaryMinus()).unaryMinus();
    }

    public void setYaw(double angleDeg) {
        yawAdjustmentAngle = getUnadjustedYaw().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
    }

    public void setRoll(double angleDeg) {
        rollAdjustmentAngle = getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
    }

    public void setPitch(double angleDeg) {
        pitchAdjustmentAngle = getUnadjustedPitch().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
    }

    public Rotation2d getUnadjustedYaw() {
        return Rotation2d.fromDegrees(mGyro.getYaw().getValue().in(Units.Degrees));
    }

    public Rotation2d getUnadjustedPitch() {
        return Rotation2d.fromDegrees(mGyro.getRoll().getValue().in(Units.Degrees));
    }

    public Rotation2d getUnadjustedRoll() {
        return Rotation2d.fromDegrees(mGyro.getPitch().getValue().in(Units.Degrees));
    }
}
