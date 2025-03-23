package frc.robot.drivers;

import edu.wpi.first.wpilibj.DigitalInput;

public final class BeamBreak {
    private final DigitalInput mBreak;
    private boolean lastStatus, tripped, cleared;

    public BeamBreak(int channel) {
        mBreak = new DigitalInput(channel);
    }

    public void update() {
        boolean value = get();
        tripped = value && !lastStatus;
        cleared = !value && lastStatus;
        lastStatus = value;
    }

    public boolean get() {
        return !mBreak.get();
    }

    public boolean wasTripped() {
        return tripped;
    }

    public boolean wasCleared() {
        return cleared;
    }
}
