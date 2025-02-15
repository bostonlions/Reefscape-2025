package frc.robot;

import java.util.Map;
import static java.util.Map.entry;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
// import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Trimmer;

public final class Auton implements Sendable {
    private static Auton mInstance;
    private int commandIdx = 0;
    private static Algae algae = Algae.getInstance();
    private static Coral coral = Coral.getInstance();
    private static Elevator elevator = Elevator.getInstance();
    // private static Drive drive = Drive.getInstance();

    private static final Map<String, Command> commands = Map.ofEntries(
        entry(
            "Flip Algae",
            algae.upCommand()
            .andThen(new WaitCommand(1.))
            .andThen(algae.downCommand())
        ),
        entry(
            "Toggle Coral",
            coral.toggleCommand()
            .andThen(new WaitCommand(1.))
            .andThen(coral.toggleCommand())
        ),
        entry(
            "Elevator Up/Down",
            elevator.stepUpCommand()
            .andThen(elevator.stepUpCommand())
            .andThen(elevator.stepUpCommand())
            .andThen(new WaitCommand(3.))
            .andThen(elevator.stepDownCommand())
            .andThen(elevator.stepDownCommand())
            .andThen(elevator.stepDownCommand())
        )
    );

    private static final String[] commandNames = commands.keySet().toArray(new String[0]);
    private static final String allCommands = String.join("\n", commandNames);

    public static Auton getInstance() {
        if (mInstance == null) mInstance = new Auton();
        return mInstance;
    }

    private Auton() {
        initTrimmer();
    }

    public Command getCommand() {
        return commands.get(commandNames[commandIdx]);
    }

    private void inc(boolean up) {
        commandIdx = commandIdx + commands.size() + (up ? 1 : -1);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Auton");
        builder.addStringProperty("Command", () -> commandNames[commandIdx], null);
        builder.addStringProperty("Commands", () -> allCommands, null);
    }

    private void initTrimmer() {
        Trimmer trimmer = Trimmer.getInstance();
        trimmer.add("Auton", "Command", () -> (double) commandIdx, this::inc);
    }
}
