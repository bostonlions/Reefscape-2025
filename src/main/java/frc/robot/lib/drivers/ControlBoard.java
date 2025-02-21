package frc.robot.lib.drivers;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Ports;
import frc.robot.subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class ControlBoard {
    private static ControlBoard mInstance = null;
    public final CustomXboxController operator;
    public final GenericHID driver;
    private final double speedFactor;
    private final double kSwerveDeadband;

    public static ControlBoard getInstance() {
        if (mInstance == null) mInstance = new ControlBoard();
        return mInstance;
    }

    private ControlBoard() {
        driver = new GenericHID(Ports.DRIVER_CONTROL);
        operator = new CustomXboxController(Ports.OPERATOR_CONTROL);
        speedFactor = ControllerConstants.kInputClipping;
        kSwerveDeadband = ControllerConstants.stickDeadband;
    }

    /** Driver method */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = 0;
        double strafeAxis = 0;
        if (ControllerConstants.isMambo) {
            forwardAxis = driver.getRawAxis(2);
            strafeAxis = driver.getRawAxis(1);
        } else {
            forwardAxis = getRightThrottle();
            strafeAxis = getRightYaw();
        }

        forwardAxis *= speedFactor;
        strafeAxis *= speedFactor;

        SmartDashboard.putNumber("Raw Y", forwardAxis);
        SmartDashboard.putNumber("Raw X", strafeAxis);

        forwardAxis = ControllerConstants.invertYAxis ? forwardAxis : -forwardAxis;
        strafeAxis = ControllerConstants.invertXAxis ? strafeAxis : -strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < kSwerveDeadband) return new Translation2d(); else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.getX(), tAxes.getY());
            Translation2d deadband_vector = new Translation2d(kSwerveDeadband, deadband_direction);

            double scaled_x = MathUtil.applyDeadband(forwardAxis, Math.abs(deadband_vector.getX()));
            double scaled_y = MathUtil.applyDeadband(strafeAxis, Math.abs(deadband_vector.getY()));
            return new Translation2d(scaled_x, scaled_y).times(Drive.getInstance().getKinematicLimits().kMaxDriveVelocity);
        }
    }

    /** Driver method */
    public double getSwerveRotation() {
        double rotAxis = ControllerConstants.isMambo ? driver.getRawAxis(3) : getLeftYaw();
        rotAxis = ControllerConstants.invertRAxis ? rotAxis : -rotAxis;
        rotAxis *= speedFactor;

        if (Math.abs(rotAxis) < kSwerveDeadband) return 0.;
        return Drive.getInstance().getKinematicLimits().kMaxAngularVelocity *
            (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband)) / (1 - kSwerveDeadband);
    }

    /** Non-mambo controller. Returns positions from -1 to 1 */
    private double getLeftYaw() {
        double leftYaw = driver.getRawAxis(ControllerConstants.leftXAxis);

        if (leftYaw != 0) leftYaw -= ControllerConstants.LeftYawZero;

        if (leftYaw > kSwerveDeadband) leftYaw /= (ControllerConstants.LeftYawHigh +
            (ControllerConstants.isC1 ? -ControllerConstants.LeftYawZero : ControllerConstants.LeftYawZero));
        else if (leftYaw < -kSwerveDeadband) leftYaw /= (ControllerConstants.LeftYawLow +
            ControllerConstants.LeftYawZero);
        return MathUtil.clamp(leftYaw, -1, 1);
    }

    /** Non-mambo controller. Returns positions from -1 to 1 */
    private double getRightThrottle() {
        double rightThrottle = driver.getRawAxis(ControllerConstants.rightYAxis);

        if (rightThrottle != 0) rightThrottle = rightThrottle - ControllerConstants.RightThrottleZero;

        if (rightThrottle > (ControllerConstants.isC1 ? kSwerveDeadband : 0.102))
            rightThrottle /= (ControllerConstants.RightThrottleHigh + (ControllerConstants.isC1 ?
                -ControllerConstants.RightThrottleZero : ControllerConstants.RightThrottleZero));
        else if (rightThrottle < -kSwerveDeadband) rightThrottle /= (ControllerConstants.RightThrottleLow
            + ControllerConstants.RightThrottleZero);
        return MathUtil.clamp(rightThrottle, -1, 1);
    }

    /** Non-mambo controller. Returns positions from -1 to 1 */
    private double getRightYaw() {
        double rightYaw = driver.getRawAxis(ControllerConstants.rightXAxis);

        if (rightYaw != 0) rightYaw -= ControllerConstants.RightYawZero;

        if (rightYaw > kSwerveDeadband) rightYaw /= (ControllerConstants.RightYawHigh -
            ControllerConstants.RightYawZero);
        else if (rightYaw < -kSwerveDeadband) rightYaw /= (ControllerConstants.RightYawLow +
            ControllerConstants.RightYawZero);
        return MathUtil.clamp(rightYaw, -1, 1);
    }
}
