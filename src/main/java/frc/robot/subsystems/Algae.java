package frc.robot.subsystems;

// import edu.wpi.first.units.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Ports;
// import frc.robot.lib.loops.ILooper;
// import frc.robot.lib.loops.Loop;
// import frc.robot.lib.Util.Conversions;
import frc.robot.Constants.AlgaeConstants;

public class Algae extends SubsystemBase {

    private static Algae mInstance;
    private TalonFX mDriveMotor;
    private TalonFX mAngleMotor;
    private DigitalInput mBeamBreak;
    private CANcoder mCANcoder;

    private mPeriodicIO mPeriodicIO = new mPeriodicIO();

    public static Algae getInstance() {
        if (mInstance == null) {
            mInstance = new Algae();
        }
        return mInstance;
    }

    private Algae() {
        mDriveMotor = new TalonFX(Ports.ALGAE_DRIVE, Ports.CANBUS_OPS);
        mDriveMotor.getConfigurator().apply(AlgaeConstants.driveMotorConfig());
        mAngleMotor = new TalonFX(Ports.ALGAE_ANGLE, Ports.CANBUS_OPS);
        mAngleMotor.getConfigurator().apply(AlgaeConstants.angleMotorConfig());

        mAngleMotor.setPosition(0);
        mBeamBreak = new DigitalInput(Ports.ALGAE_BEAMBREAK);
        mCANcoder = new CANcoder(Ports.ALGAE_CANCODER, Ports.CANBUS_OPS);
        mCANcoder.getConfigurator().apply(AlgaeConstants.cancoderConfig());
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

        public boolean beambreak = false;

        // Outputs
        public double demand = 0;
        public ControlModeState mControlModeState;
    }

    private enum ControlModeState {
        MOTION_MAGIC,
        OPEN_LOOP
    }

    @Override
    public void periodic() {
        // TODO: read motors & sensors to mPeriodicIO
        mPeriodicIO.beambreak = !mBeamBreak.get();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        // TODO: builder.addDoubleProperty(.....) etc
        builder.addBooleanProperty("Algae Beambreak", () -> mPeriodicIO.beambreak, null);
    }
}
