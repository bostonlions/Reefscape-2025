package frc.robot;

import java.util.Map;
import java.util.function.Supplier;
import static java.util.Map.entry;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.Constants.ElevatorConstants.Position;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Trimmer;

public final class Auton extends SubsystemBase {
    private static Auton mInstance = null;
    private static final Algae algae = Algae.getInstance();
    private static final Coral coral = Coral.getInstance();
    private static final Elevator elevator = Elevator.getInstance();
    private static final Drive drive = Drive.getInstance();
    private static final Map<String, Command> commands = Map.ofEntries(
        entry(
            "Drive Test",
            drive.followPathCommand("backAwayFromReef", true)
            .andThen(debug(() -> "Done driving at " + (System.currentTimeMillis() % 60000) + "ms after the start of this minute"))
            //.andThen(coral.toggleCommand())
            //.andThen(sleep(1.))
            //.andThen(coral.toggleCommand())
            //.andThen(elevator.stepUpCommand())  // shouldn't need this but for now it seems to help
            //.andThen(elevator.stepToCommand(Position.L2))
            //.andThen(elevator.stepToCommand(Position.LOAD))
            // .andThen(debug(() -> "About to run path test2"))
            // .andThen(drive.followPathCommand("test2", false))
            // .andThen(debug(() -> "test2 path has been run"))
            // .andThen(algae.upCommand())
            // .andThen(sleep(1))
            // .andThen(algae.toggleDriveCommand())
            // .andThen(sleep(2))
            // .andThen(algae.toggleDriveCommand())
        ),
        entry(
            "Parallel example",
            new ParallelCommandGroup(
                drive.followPathCommand("backAwayFromReef", true),
                sleep(1.)
                .andThen(elevator.stepToCommand(Position.L2))
                .andThen(algae.toggleDriveCommand())
                .andThen(coral.toggleCommand())
                .andThen(sleep(1.))
                .andThen(elevator.stepToCommand(Position.LOAD))
            )
        )
        // entry(
        //     "Flip Algae 2",
        
        //     algae.upCommand()
        //     .andThen(sleep(1.))
        //     .andThen(debug(() -> "Flipped up 2 at " + (System.currentTimeMillis() % 60000) + "ms after the start of this minute"))
        //     .andThen(algae.downCommand())
        //     .andThen(debug(() -> "Flipped down 2 at " + (System.currentTimeMillis() % 60000) + "ms after the start of this minute"))

        // )
       //,
        // entry(
        //     "Toggle Coral",
        //     coral.toggleCommand()
        //     .andThen(sleep(1.))
        //     .andThen(coral.toggleCommand())
        // ),
        // entry(
        //     "Elevator Up/Down",
        //     elevator.stepUpCommand()
        //     .andThen(elevator.stepToCommand(Position.L3))
        //     .andThen(sleep(3.))
        //     .andThen(elevator.stepToCommand(Position.LOAD))
        // ),
        // entry(
        //     "Turn around",
        //     drive.headingCommand(90)
        //     .andThen(sleep(2.))
        //     .andThen(drive.headingCommand(180))
        //     .andThen(sleep(2.))
        //     .andThen(drive.headingCommand(270))
        //     .andThen(drive.headingCommand(0))
        // )
    );

    private int commandIdx = 0;

    private static final String[] commandNames = commands.keySet().toArray(new String[0]);
    private static final String allCommands = String.join("\n", commandNames);

    public static Auton getInstance() {
        if (mInstance == null) mInstance = new Auton();
        return mInstance;
    }

    private Auton() {
        initTrimmer();
    }

    private static Command sleep(double secs) {
        return new WaitCommand(secs);
    }

    private static Command debug(Supplier<String> s) {
        return new InstantCommand(() -> System.out.println(s.get()));
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
        Trimmer.getInstance().add("Auton", "Command", () -> (double) commandIdx, this::inc);
    }
}
