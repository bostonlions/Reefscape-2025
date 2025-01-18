package frc.robot.subsystems;

// import edu.wpi.first.units.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Ports;
// import frc.robot.lib.loops.ILooper;
// import frc.robot.lib.loops.Loop;
// import frc.robot.lib.Util.Conversions;
import frc.robot.Constants.AlgaeConstants;

public class Algae extends Subsystem {

    private static Algae mInstance;
    private TalonFX mDriveMotor;
    private TalonFX mAngleMotor;

    // private mPeriodicIO mPeriodicIO = new mPeriodicIO();

    public static Algae getInstance() {
        if (mInstance == null) {
            mInstance = new Algae();
        }
        return mInstance;
    }

    private Algae() {
        mDriveMotor = new TalonFX(Ports.ALGAE_DRIVE, Ports.CANBUS_UPPER);
        mDriveMotor.getConfigurator().apply(AlgaeConstants.driveMotorConfig());
        mAngleMotor = new TalonFX(Ports.ALGAE_ANGLE, Ports.CANBUS_UPPER);
        mAngleMotor.getConfigurator().apply(AlgaeConstants.angleMotorConfig());

        mAngleMotor.setPosition(0);
    }

    public static class mPeriodicIO {
        // TODO - 2 motors
        // Inputs
        public double timestamp = 0.0;
        public double targetVelocity = 0.0;
        public double position_degrees = 0.0;
        public double velocity_rps = 0.0;

        public double current = 0.0;
        public double output_voltage = 0.0;

        // Outputs
        public double demand = 0;
        public ControlModeState mControlModeState;
    }

    private enum ControlModeState {
        MOTION_MAGIC,
        OPEN_LOOP
    }
}
