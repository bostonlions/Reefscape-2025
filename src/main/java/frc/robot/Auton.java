package frc.robot;

import java.util.Map;
import static java.util.Map.entry;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants.ElevatorConstants.Position;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Trimmer;

public final class Auton extends SubsystemBase {
    private static Auton mInstance;
    private int commandIdx = 0;
    private static Algae algae = Algae.getInstance();
    private static Coral coral = Coral.getInstance();
    private static Elevator elevator = Elevator.getInstance();
    private static Drive drive = Drive.getInstance();

    private static final Map<String, Command> commands = Map.ofEntries(
        entry(
            "Flip Algae",
            algae.upCommand()
            .andThen(_wait(1.))
            .andThen(algae.downCommand())
        ),
        entry(
            "Toggle Coral",
            coral.toggleCommand()
            .andThen(_wait(1.))
            .andThen(coral.toggleCommand())
        ),
        entry(
            "Elevator Up/Down",
            elevator.stepUpCommand()
            .andThen(elevator.stepToCommand(Position.L3))
            .andThen(_wait(3.))
            .andThen(elevator.stepToCommand(Position.LOAD))
        ),
        entry(
            "Turn around",
            drive.headingCommand(90)
            .andThen(_wait(2.))
            .andThen(drive.headingCommand(180))
            .andThen(_wait(2.))
            .andThen(drive.headingCommand(270))
            .andThen(drive.headingCommand(0))
        ),
        entry(
            "Drive Test",
            _wait(2) // need to bring in AutoTrajectory stuff
            // drive.trajectoryCommand(..., 0)
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

    private static Command _wait(double secs) {
        return new WaitCommand(secs);
    }

    public Command getCommand() {
        return commands.get(commandNames[commandIdx]);
    }

    private void inc(boolean up) {
        commandIdx = (commandIdx + commands.size() + (up ? 1 : -1)) % commands.size();
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
