package frc.robot.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import frc.robot.lib.Util;

public class ModuleState extends SwerveModulePosition {

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

    public ModuleState optimize(Rotation2d currentAngle) {
        double targetAngle = Util.placeInAppropriate0To360Scope(currentAngle.getDegrees(), this.angle.getDegrees());
        double targetSpeed = this.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90){
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return ModuleState.fromSpeeds(Rotation2d.fromDegrees(targetAngle), targetSpeed);
      }
}