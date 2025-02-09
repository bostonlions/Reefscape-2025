package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Trimmer extends SubsystemBase {
    private static Trimmer mInstance;

    private List<String> subsystems;
    private List<List<Item>> items;
    private int currentSubsystemIndex;
    private String currentSubsystem;
    private int currentItemIndex;
    private Item currentItem;
    private String[] subsystemsArray;
    private String[] itemsArray;

    public static Trimmer getInstance() {
        if (mInstance == null) mInstance = new Trimmer();
        return mInstance;
    }

    private Trimmer() {
        subsystems = new ArrayList<>();
        items = new ArrayList<>();
        currentSubsystemIndex = 0;
        currentItemIndex = 0;
        currentSubsystem = "-";
        currentItem = new Item("-", () -> 0.0, (v) -> {});
    }

    public void add(String subsystem, String name, DoubleSupplier getter, BooleanConsumer inc) {
        Item item = new Item(name, getter, inc);
        int ssIndex = subsystems.indexOf(subsystem);
        if (ssIndex == -1) {
            ssIndex = subsystems.size();
            subsystems.add(subsystem);
            items.add(new ArrayList<>());
            subsystemsArray = subsystems.toArray(new String[0]);
        }
        items.get(ssIndex).add(item);
        setItem();
    }

    private void setItem() {
        currentSubsystem = subsystems.get(currentSubsystemIndex);
        currentItem = items.get(currentSubsystemIndex).get(currentItemIndex);
        itemsArray = new String[items.get(currentSubsystemIndex).size()];
        for (int i = 0; i < itemsArray.length; i++) itemsArray[i] = items.get(currentSubsystemIndex).get(i).name;
    }

    public void nextSubsystem() {
        currentSubsystemIndex = (currentSubsystemIndex + 1) % subsystems.size();
        currentItemIndex = 0;
        setItem();
    }

    public void nextItem() {
        currentItemIndex = (currentItemIndex + 1) % items.get(currentSubsystemIndex).size();
        setItem();
    }

    public void incrementItem() {
        items.get(currentSubsystemIndex).get(currentItemIndex).inc.accept(true);
    }

    public void decrementItem() {
        items.get(currentSubsystemIndex).get(currentItemIndex).inc.accept(false);
    }

    public class Item {
        public String name;
        public DoubleSupplier getter;
        public BooleanConsumer inc;
        public Runnable dec;

        public Item(String name, DoubleSupplier getter, BooleanConsumer inc) {
            this.name = name;
            this.getter = getter;
            this.inc = inc;
        }
    }

    /**
     * find the next larger increment of a value
     * @param initial
     * @param min the minimum nonzero value to return
     * @param inc fraction of the value to add on
     * @param up are we going up (true) or down (false)
     */
    public static double increment(double initial, double min, double inc, boolean up) {
        if (up) {
            if (initial < min) return min;
            return initial * (1. + inc);
        }
        if (initial <= min) return 0;
        return initial / (1. + inc);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Trimmer");
        builder.addStringProperty("Subsystem", () -> currentSubsystem, null);
        builder.addStringProperty("Item", () -> currentItem.name, null);
        builder.addDoubleProperty("Value", () -> currentItem.getter.getAsDouble(), null);
        builder.addStringArrayProperty("Subsystems", () -> subsystemsArray, null);
        builder.addStringArrayProperty("Items", () -> itemsArray, null);
    }
}
