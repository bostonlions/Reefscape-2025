package frc.robot;

public final class Ports {
    public static final String CANBUS_DRIVE = "Skibidi";
    public static final String CANBUS_OPS = "";

    public static final int DRIVER_CONTROL = 2;
    public static final int OPERATOR_CONTROL = 1;

    /**
     * Swerve Modules - as viewed from the bottom:
     * FL FR
     * BL BR
     */

    public static final int PIGEON = 13;

    public static final int FL_DRIVE = 9;
    public static final int FL_ROTATION = 12;
    public static final int FL_CANCODER = 2;

    public static final int FR_DRIVE = 7;
    public static final int FR_ROTATION = 10;
    public static final int FR_CANCODER = 1;

    public static final int BL_DRIVE = 6;
    public static final int BL_ROTATION = 11;
    public static final int BL_CANCODER = 4;

    public static final int BR_DRIVE = 5;
    public static final int BR_ROTATION = 8;
    public static final int BR_CANCODER = 3;

    public static final int ELEVATOR_A = -1;
    public static final int ELEVATOR_B = -1;

    public static final int CLIMBER_HOOK_DRIVE = -1;
    public static final int CLIMBER_HOOK_CANCODER = -1;

    public static final int CORAL_DRIVE = -1;
    public static final int CORAL_BEAM_BREAK = -1;  // maybe one more?

    public static final int ALGAE_DRIVE = -1;
    public static final int ALGAE_ANGLE = -1;
    public static final int ALGAE_CANCODER = -1;
    public static final int ALGAE_BEAMBREAK = -1;
}