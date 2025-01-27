package frc.robot.controlboard;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Ports;
// import frc.robot.controlboard.CustomXboxController.Axis;
// import frc.robot.controlboard.CustomXboxController.Button;
// import frc.robot.controlboard.CustomXboxController.Side;
import frc.robot.subsystems.Drive;
import frc.robot.lib.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControlBoard {
    private final double kSwerveDeadband = ControllerConstants.stickDeadband;

    private final int kDpadUp = 0;
    private final int kDpadRight = 90;
    private final int kDpadDown = 180;
    private final int kDpadLeft = 270;

    // private boolean leftBumperBoolean = false;
    // private boolean passNoteAllignBoolean = false;
    // private boolean podiumAllignBoolean = false;

    private static ControlBoard mInstance = null;

    int tagLastChased = -1;

    public static ControlBoard getInstance() {
        if (mInstance == null) mInstance = new ControlBoard();
        return mInstance;
    }

    private final GenericHID m_driver;
    public final CustomXboxController operator;

    private ControlBoard() {
        m_driver = new GenericHID(Ports.DRIVER_CONTROL);
        operator = new CustomXboxController(Ports.OPERATOR_CONTROL);
    }

    /* DRIVER METHODS */
    public Translation2d getSwerveTranslation() {
        double forwardAxis = 0;
        double strafeAxis = 0;
        if (ControllerConstants.isMambo) {
            forwardAxis = m_driver.getRawAxis(2);
            strafeAxis = m_driver.getRawAxis(1);
        } else {
            forwardAxis = getRightThrottle();
            strafeAxis = getRightYaw();
        }

        SmartDashboard.putNumber("Raw Y", forwardAxis);
        SmartDashboard.putNumber("Raw X", strafeAxis);

        forwardAxis = ControllerConstants.invertYAxis ? forwardAxis : -forwardAxis;
        strafeAxis = ControllerConstants.invertXAxis ? strafeAxis : -strafeAxis;

        Translation2d tAxes = new Translation2d(forwardAxis, strafeAxis);

        if (Math.abs(tAxes.getNorm()) < kSwerveDeadband) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(tAxes.getX(), tAxes.getY());
            Translation2d deadband_vector = new Translation2d(kSwerveDeadband, deadband_direction);

            double scaled_x = Util.scaledDeadband(forwardAxis, 1.0, Math.abs(deadband_vector.getX()));
            double scaled_y = Util.scaledDeadband(strafeAxis, 1.0, Math.abs(deadband_vector.getY()));
            return new Translation2d(scaled_x, scaled_y)
                    .times(Drive.getInstance().getKinematicLimits().kMaxDriveVelocity);
        }

    }

    public double getSwerveRotation() {
        double rotAxis = ControllerConstants.isMambo ? m_driver.getRawAxis(3) : getLeftYaw();
        rotAxis = ControllerConstants.invertRAxis ? rotAxis : -rotAxis;

        if (Math.abs(rotAxis) < kSwerveDeadband) return 0.0;

        return Drive.getInstance().getKinematicLimits().kMaxAngularVelocity *
            (rotAxis - (Math.signum(rotAxis) * kSwerveDeadband)) / (1 - kSwerveDeadband);
    }

    /* right switch up */
    public boolean zeroGyro() {
        // SmartDashboard.putBoolean("Zero Gyro", m_driver.getRawButton(2));
        return m_driver.getRawButton(2);// (m_driver.getRawAxis(6)<-0.3);
    }

    public enum SwerveCardinal {
        NONE(0),

        FORWARDS(0),
        LEFT(270),
        RIGHT(90),
        BACKWARDS(180);

        public final double degrees;

        SwerveCardinal(double degrees) {
            this.degrees = degrees;
        }
    }

    public SwerveCardinal getSwerveSnap() {
        // CARDINAL SNAPS

        switch (operator.getController().getPOV()) {
            case kDpadUp:
                return SwerveCardinal.FORWARDS;
            case kDpadLeft:
                return SwerveCardinal.RIGHT;
            case kDpadRight:
                return SwerveCardinal.LEFT;
            case kDpadDown:
                return SwerveCardinal.BACKWARDS;
            default:
                return SwerveCardinal.NONE;
        }
    }

    /** far right switch */
    public boolean snapToTarget() { // DISABLED
        return false;
        // m_driver.getRawAxis(4)<-0.25 || autoSnap;

        // if (m_driver.getRawAxis(4)<-0.25 && leftSwitchReset){
        // leftSwitchReset = false;
        // return true;
        // }
        // else if(!(m_driver.getRawAxis(4)<-0.25)){
        // leftSwitchReset = true;
        // }
        // return false;
    }

    public void setAutoSnapToTarget(boolean snap) {
        //would change the angle of pivot if there was a limelight
    }

    // public boolean farLeftSwitchUp(){//DISABLED
    // return false; //m_driver.getRawAxis(4)<-0.25;
    // }

    /** right bumper */
    // public boolean allignWithHumanPlayer() {
    //     if (leftBumperBoolean != m_driver.getRawButton(1)) {
    //         leftBumperBoolean = !leftBumperBoolean;
    //         return true;
    //     }

    //     return false;
    //     // ;
    // }

    // public boolean passNoteFromMidAllign() { // triggers once every click up
    //     if (!(m_driver.getRawAxis(6) < -0.25)) {
    //         passNoteAllignBoolean = true;
    //     } else if (m_driver.getRawAxis(6) < -0.25 && passNoteAllignBoolean) {
    //         passNoteAllignBoolean = false;
    //         return true;
    //     }
    //     return false;
    // }

    // public boolean passNoteFromMid() {
    //     return (m_driver.getRawAxis(6) < -0.25);
    // }

    // Far right switch
    // public boolean shootFromPodiumAllign() { // triggers once every click up
    //     if (!(m_driver.getRawAxis(4) < -0.25)) {
    //         podiumAllignBoolean = true;
    //     } else if (m_driver.getRawAxis(4) < -0.25 && podiumAllignBoolean) {
    //         podiumAllignBoolean = false;
    //         return true;
    //     }
    //     return false;
    // }

    // public boolean farLeftSwitchUp(){
    //    return (m_driver.getRawAxis(4)<-0.25);
    // }

    // public boolean shootFromOppositePodiumAllign() { // triggers once every click up
    //     if (!(m_driver.getRawAxis(5) < -0.25)) {
    //         podiumAllignBoolean = true;
    //     } else if (m_driver.getRawAxis(5) < -0.25 && podiumAllignBoolean) {
    //         podiumAllignBoolean = false;
    //         return true;
    //     }
    //     return false;
    // }

    // public boolean shootFromPodium() {
    //     return (m_driver.getRawAxis(4) < -0.25)||(m_driver.getRawAxis(5) < -0.25);
    // }

    // public double pivotPercentOutput() {
    //     return operator.getAxis(Side.LEFT, Axis.Y);
    // }

    // public boolean pivotUp() {
    //     return operator.getButton(Button.Y);
    // }

    // public boolean pivotDown() {
    //     return operator.getButton(Button.A);
    // }

    // public boolean elevatorUp() {
    //     return operator.getButton(Button.RB);
    // }

    // public boolean elevatorDown() {
    //     return operator.getButton(Button.LB);
    // }

    // public boolean zeroElevator() {
    //     return operator.getButton(Button.START);
    // }

    // public double elevatorPercentOutput() {
    //     return operator.getAxis(Side.RIGHT, Axis.Y);
    // }

    // public double wristPercentOutput() {
    //     return operator.getAxis(Side.RIGHT, Axis.X);
    // }

    // public boolean endEffectorIntake() {
    //     return operator.getButton(Button.Y);
    // }

    // public boolean endEffectorOuttake() {
    //     return operator.getButton(Button.A);
    // }

    // Align swerve drive with target
    // public boolean getWantChase() {
    //     return false;// (m_driver.getRawButton(9)||(operator.getButton(Button.Y))||(operator.getButton(Button.B))||(operator.getButton(Button.A))||(operator.getButton(Button.X)));
    // }

    // public int tagToChase() {
    //     if (chaseTag1()) {
    //         tagLastChased = 1;
    //         return 1;
    //     } else if (chaseTag2()) {
    //         tagLastChased = 2;
    //         return 2;
    //     } else if (chaseTag3()) {
    //         tagLastChased = 3;
    //         return 3;
    //     } else if (chaseTag4()) {
    //         tagLastChased = 4;
    //         return 4;
    //     } else {
    //         return tagLastChased;
    //     }
    // }

    // public boolean chaseNearest() {
    //     return m_driver.getRawButton(9);
    // }

    // public boolean chaseTag1() {
    //     return (operator.getButton(Button.Y));
    // }

    // public boolean chaseTag2() {
    //     return (operator.getButton(Button.B));
    // }

    // public boolean chaseTag3() {
    //     return (operator.getButton(Button.A));
    // }

    // public boolean chaseTag4() {
    //     return (operator.getButton(Button.X));
    // }

    // // Locks wheels in X formation
    public boolean getBrake() {
        SmartDashboard.putNumber("Get Brake", m_driver.getRawAxis(4));
        return false;// (m_driver.getRawAxis(4)<-0.3); //m_driver.getRawButton(4); //far left switch
    }

    // // Intake Controls
    // public boolean getIntake() {
    // return operator.getTrigger(CustomXboxController.Side.RIGHT);
    // }

    // public boolean getReject() {
    // return operator.getTrigger(CustomXboxController.Side.LEFT);
    // }

    /* Non-mambo controller */

    // Returns positions from -1 to 1
    private double getLeftYaw() {
        double leftYaw = m_driver.getRawAxis(ControllerConstants.leftXAxis);

        if (leftYaw != 0) {
            leftYaw = leftYaw - ControllerConstants.LeftYawZero;
        }

        if (leftYaw > kSwerveDeadband) {
            leftYaw = (leftYaw / (ControllerConstants.LeftYawHigh
                    + (ControllerConstants.isC1
                            ? -ControllerConstants.LeftYawZero
                            : ControllerConstants.LeftYawZero)));
        } else if (leftYaw < -kSwerveDeadband) {
            leftYaw = (leftYaw / (ControllerConstants.LeftYawLow
                    + ControllerConstants.LeftYawZero));
        }

        // SmartDashboard.putNumber("remote leftYaw", leftYaw);
        return Util.limit(leftYaw, 1);
    }

    // Returns positions from -1 to 1
    // private double getLeftThrottle() {
    // double leftThrottle = m_driver.getRawAxis(ControllerConstants.leftYAxis);

    // if (leftThrottle != 0){
    // leftThrottle = leftThrottle -
    // ControllerConstants.LeftThrottleZero;
    // }

    // if (leftThrottle > kSwerveDeadband){
    // leftThrottle = (leftThrottle /
    // (ControllerConstants.LeftThrottleHigh +
    // ControllerConstants.LeftThrottleZero));
    // }
    // else if (leftThrottle < -kSwerveDeadband){
    // leftThrottle = (leftThrottle /
    // (ControllerConstants.LeftThrottleLow +
    // ControllerConstants.LeftThrottleZero));
    // }

    // if (leftThrottle>1){
    // leftThrottle = 1;
    // }

    // if (leftThrottle<-1){
    // leftThrottle = -1;
    // }

    // //SmartDashboard.putNumber("remote leftThrottle", leftThrottle);
    // return leftThrottle;
    // }

    private double getRightThrottle() {
        double rightThrottle = m_driver.getRawAxis(ControllerConstants.rightYAxis);

        if (rightThrottle != 0)
            rightThrottle = rightThrottle - ControllerConstants.RightThrottleZero;

        if (rightThrottle > (ControllerConstants.isC1 ? kSwerveDeadband : 0.102)) {
            rightThrottle = (rightThrottle / (ControllerConstants.RightThrottleHigh
                    + (ControllerConstants.isC1
                            ? -ControllerConstants.RightThrottleZero
                            : ControllerConstants.RightThrottleZero)));
        } else if (rightThrottle < -kSwerveDeadband) {
            rightThrottle = (rightThrottle / (ControllerConstants.RightThrottleLow
                    + ControllerConstants.RightThrottleZero));
        }

        // SmartDashboard.putNumber("remote rightThrottle", rightThrottle);
        return Util.limit(rightThrottle, 1);
    }

    private double getRightYaw() {
        double rightYaw = m_driver.getRawAxis(ControllerConstants.rightXAxis);

        if (rightYaw != 0)
            rightYaw = rightYaw - ControllerConstants.RightYawZero;

        if (rightYaw > kSwerveDeadband) {
            rightYaw = (rightYaw / (ControllerConstants.RightYawHigh
                    + -ControllerConstants.RightYawZero));
        } else if (rightYaw < -kSwerveDeadband) {
            rightYaw = (rightYaw / (ControllerConstants.RightYawLow
                    + ControllerConstants.RightYawZero));
        }

        // SmartDashboard.putNumber("remote rightYaw", rightYaw);
        return Util.limit(rightYaw, 1);
    }
}
