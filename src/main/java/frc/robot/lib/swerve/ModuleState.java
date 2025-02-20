package frc.robot.lib.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public final class ModuleState extends SwerveModulePosition {
    public double speedMetersPerSecond;

    public ModuleState() {
        super(0.0, new Rotation2d());
        speedMetersPerSecond = 0.0;
    }

    public ModuleState(double distanceMeters, Rotation2d angle, double speedMetersPerSecond) {
        super(distanceMeters, angle);
        this.speedMetersPerSecond = speedMetersPerSecond;
    }

    public static ModuleState fromSpeeds(Rotation2d angle, double speedMetersPerSecond) {
        return new ModuleState(Double.NaN, angle, speedMetersPerSecond);
    }

    public ModuleState optimize(Rotation2d currentAngle) { // TODO: use this
        double currentDeg = currentAngle.getDegrees();
        double targetAngle = MathUtil.inputModulus(angle.getDegrees(), currentDeg - 180, currentDeg + 180);
        double targetSpeed = this.speedMetersPerSecond;
        double delta = targetAngle - currentDeg;
        if (Math.abs(delta) > 90) {
            targetSpeed *= -1;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return ModuleState.fromSpeeds(Rotation2d.fromDegrees(targetAngle), targetSpeed);
    }
}
