package frc.robot.lib.drivers;

import edu.wpi.first.wpilibj.DigitalInput;

public final class LimitSwitch {
    private final DigitalInput mSwitch;
    private boolean lastStatus;
    private boolean tripped;
    private boolean cleared;

    public LimitSwitch(int channel) {
        mSwitch = new DigitalInput(channel);
    }

    public void update() {
        boolean value = get();
        tripped = value && !lastStatus;
        cleared = !value && lastStatus;
        lastStatus = value;
    }

    public boolean get() {
        return mSwitch.get();
    }

    public boolean wasTripped() {
        return tripped;
    }

    public boolean wasCleared() {
        return cleared;
    }
}

