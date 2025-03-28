package frc.robot.drivers;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Ports;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class ControlBoard {
    private static ControlBoard mInstance;
    public final CustomXboxController operator;
    public final GenericHID driver;
    private final double swerveDeadband;

    public static ControlBoard getInstance() {
        if (mInstance == null) mInstance = new ControlBoard();
        return mInstance;
    }

    private ControlBoard() {
        driver = new GenericHID(Ports.DRIVER_CONTROL);
        operator = new CustomXboxController(Ports.OPERATOR_CONTROL);
        swerveDeadband = ControllerConstants.stickDeadband;
    }

    /** Driver method */
    public Translation2d getSwerveTranslation() {
        double forwardAxis;
        double strafeAxis;
        if (ControllerConstants.isMambo) {
            forwardAxis = driver.getRawAxis(2);
            strafeAxis = driver.getRawAxis(1);
            double mag = Math.pow(forwardAxis * forwardAxis + strafeAxis * strafeAxis, 0.5);
            double curveFactor = Math.pow(mag, 0.25);
            forwardAxis *= curveFactor;
            strafeAxis *= curveFactor;
        } else {
            forwardAxis = getRightThrottle();
            strafeAxis = getRightYaw();
        }

        SmartDashboard.putNumber("Raw Y", forwardAxis);
        SmartDashboard.putNumber("Raw X", strafeAxis);

        if (!ControllerConstants.invertYAxis) forwardAxis = -forwardAxis;
        if (!ControllerConstants.invertXAxis) strafeAxis = -strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < swerveDeadband) return new Translation2d();

        Rotation2d deadband_direction = new Rotation2d(tAxes.getX(), tAxes.getY());
        Translation2d deadband_vector = new Translation2d(swerveDeadband, deadband_direction);

        double scaled_x = MathUtil.applyDeadband(forwardAxis, Math.abs(deadband_vector.getX()));
        double scaled_y = MathUtil.applyDeadband(strafeAxis, Math.abs(deadband_vector.getY()));
        return new Translation2d(scaled_x, scaled_y).times(SwerveConstants.SMConstFactory.SpeedAt12Volts);
    }

    /** Driver method */
    public double getSwerveRotation() {
        double rotAxis = ControllerConstants.isMambo ? driver.getRawAxis(3) : getLeftYaw();
        if (!ControllerConstants.invertRAxis) rotAxis = -rotAxis;
        return Math.abs(rotAxis) < swerveDeadband ? 0 : SwerveConstants.SMConstFactory.SpeedAt12Volts *
            (rotAxis - (Math.signum(rotAxis) * swerveDeadband)) / (1 - swerveDeadband);
    }

    /** Non-mambo controller. Returns positions from -1 to 1 */
    private double getLeftYaw() {
        double leftYaw = driver.getRawAxis(ControllerConstants.leftXAxis);

        if (leftYaw != 0) leftYaw -= ControllerConstants.LeftYawZero;

        if (leftYaw > swerveDeadband) leftYaw /= (ControllerConstants.LeftYawHigh +
            (ControllerConstants.isC1 ? -ControllerConstants.LeftYawZero : ControllerConstants.LeftYawZero));
        else if (leftYaw < -swerveDeadband) leftYaw /= (ControllerConstants.LeftYawLow +
            ControllerConstants.LeftYawZero);
        return MathUtil.clamp(leftYaw, -1, 1);
    }

    /** Non-mambo controller. Returns positions from -1 to 1 */
    private double getRightThrottle() {
        double rightThrottle = driver.getRawAxis(ControllerConstants.rightYAxis);

        if (rightThrottle != 0) rightThrottle = rightThrottle - ControllerConstants.RightThrottleZero;

        if (rightThrottle > (ControllerConstants.isC1 ? swerveDeadband : 0.102))
            rightThrottle /= (ControllerConstants.RightThrottleHigh + (ControllerConstants.isC1 ?
                -ControllerConstants.RightThrottleZero : ControllerConstants.RightThrottleZero));
        else if (rightThrottle < -swerveDeadband) rightThrottle /= (ControllerConstants.RightThrottleLow
            + ControllerConstants.RightThrottleZero);
        return MathUtil.clamp(rightThrottle, -1, 1);
    }

    /** Non-mambo controller. Returns positions from -1 to 1 */
    private double getRightYaw() {
        double rightYaw = driver.getRawAxis(ControllerConstants.rightXAxis);

        if (rightYaw != 0) rightYaw -= ControllerConstants.RightYawZero;

        if (rightYaw > swerveDeadband) rightYaw /= (ControllerConstants.RightYawHigh -
            ControllerConstants.RightYawZero);
        else if (rightYaw < -swerveDeadband) rightYaw /= (ControllerConstants.RightYawLow +
            ControllerConstants.RightYawZero);
        return MathUtil.clamp(rightYaw, -1, 1);
    }
}
