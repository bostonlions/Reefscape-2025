package frc.robot;

import java.util.Arrays;
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
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Trimmer;

public final class Auton extends SubsystemBase {
    private static Auton mInstance = null;
    private static final Algae algae = Algae.getInstance();
    private static final Coral coral = Coral.getInstance();
    private static final Elevator elevator = Elevator.getInstance();
    private static final SwerveDrive drive = SwerveDrive.getInstance();
    private static final Map<String, Command> commands = Map.ofEntries(
        // start with an empty command for safety
        entry("00 - None", debug(() -> "Autonomous started with no command chosen")
        // ), entry("01 - Position 1",
        //     drive.followPathCommand("Position1", true)
        //     .andThen(elevator.stepToCommand(Position.L4))
        //     .andThen(coral.toggleCommand())
        //     .andThen(sleep(1))
        // ), entry("02 - Position 2",
        //     drive.followPathCommand("Position2", true)
        //     .andThen(elevator.stepToCommand(Position.L4))
        //     .andThen(coral.toggleCommand())
        //     .andThen(sleep(1))
        // ), entry("03 - Position 3",
        //     drive.followPathCommand("Position3", true)
        //     .andThen(elevator.stepToCommand(Position.L4))
        //     .andThen(coral.toggleCommand())
        //     .andThen(sleep(1))
        ), entry("01 - pos2 with algae",
            new ParallelCommandGroup(
                drive.followPathCommand("Position2", true),
                algae.upCommand()
                .andThen(sleep(1))
                .andThen(elevator.stepToCommand(Position.L4))
            )
            .andThen(sleep(.5)) // give time for robot to stabilize after bringing elevator up
            .andThen(coral.toggleCommand())
            .andThen(sleep(.3))
            .andThen(new ParallelCommandGroup(
                drive.followPathCommand("FrontCoralToAlgae", false),
                elevator.stepToCommand(Position.LOAD)
            ))
            .andThen(sleep(.5))
            .andThen(algae.toggleDriveCommand())
            .andThen(drive.followPathCommand("MoveToFrontAlgae", false))
            .andThen(sleep(1.3))
            .andThen(new ParallelCommandGroup(
                drive.followPathCommand("FrontReefToBarge", false),
                sleep(.15)
                .andThen(elevator.stepToCommand(Position.BARGE))
            ))
            .andThen(sleep(.5))
            .andThen(algae.toggleDriveCommand())
            .andThen(sleep(.3))
            .andThen(new ParallelCommandGroup(
                drive.followPathCommand("BackUpFromBarge", false),
                sleep(0.5)
                .andThen(elevator.stepToCommand(Position.LOAD))
            ))
        ), entry("02 - pos1 with algae",
            new ParallelCommandGroup(
                drive.followPathCommand("Position1", true),
                algae.upCommand()
                .andThen(sleep(1))
                .andThen(elevator.stepToCommand(Position.L4))
            )
            .andThen(coral.toggleCommand())
            .andThen(sleep(.3))
            .andThen(new ParallelCommandGroup(
                drive.followPathCommand("RightCoralToAlgae", false),
                elevator.stepToCommand(Position.L2)
            ))
            .andThen(algae.toggleDriveCommand())
            .andThen(sleep(1.3))
            .andThen(new ParallelCommandGroup(
                drive.followPathCommand("RightReefToBarge", false),
                elevator.stepToCommand(Position.BARGE)
                .andThen(sleep(.5))
            ))
            .andThen(algae.toggleDriveCommand())
            .andThen(sleep(.25))
            .andThen(new ParallelCommandGroup(
                drive.followPathCommand("BackUpFromBarge", false),
                sleep(0.5)
                .andThen(elevator.stepToCommand(Position.LOAD))
            ))
        ), entry("03 - pos3 with algae",
            new ParallelCommandGroup(
                drive.followPathCommand("Position3", true),
                algae.upCommand()
                .andThen(sleep(1))
                .andThen(elevator.stepToCommand(Position.L4))
            )
            .andThen(coral.toggleCommand())
            .andThen(sleep(.3))
            .andThen(new ParallelCommandGroup(
                drive.followPathCommand("RightReefCoralToAlgae", false),
                elevator.stepToCommand(Position.L2)
            ))
            .andThen(algae.toggleDriveCommand())
            .andThen(sleep(1.3))
            .andThen(new ParallelCommandGroup(
                drive.followPathCommand("LeftReefToBarge", false),
                elevator.stepToCommand(Position.BARGE)
                .andThen(sleep(.5))
            ))
            .andThen(algae.toggleDriveCommand())
            .andThen(sleep(.25))
            .andThen(new ParallelCommandGroup(
                drive.followPathCommand("BackUpFromBarge", false),
                sleep(0.5)
                .andThen(elevator.stepToCommand(Position.LOAD))
            ))
        ), entry("04 - pos2 almost 2 algae",
            new ParallelCommandGroup(
                drive.followPathCommand("Position2", true),
                algae.upCommand()
                .andThen(sleep(1))
                .andThen(elevator.stepToCommand(Position.L4))
            )
            .andThen(coral.toggleCommand())
            .andThen(sleep(.3))
            .andThen(new ParallelCommandGroup(
                drive.followPathCommand("FrontCoralToAlgae", false),
                elevator.stepToCommand(Position.LOAD)
            ))
            .andThen(algae.toggleDriveCommand())
            .andThen(drive.followPathCommand("MoveToFrontAlgae", false))
            .andThen(new ParallelCommandGroup(
                drive.followPathCommand("FrontReefToBarge", false),
                sleep(.25)
                .andThen(elevator.stepToCommand(Position.BARGE))
            ))
            .andThen(algae.toggleDriveCommand())
            .andThen(sleep(.15))
            .andThen(new ParallelCommandGroup(
                drive.followPathCommand("BargeToRightReef", false),
                sleep(.25)
                .andThen(elevator.stepToCommand(Position.L2))
            ))
            // .andThen(drive.followPathCommand("PullUpRightReef", false))
            // .andThen(algae.toggleDriveCommand())
            // .andThen(new ParallelCommandGroup(
            //     drive.followPathCommand("rightReefToBarge", false),
            //     sleep(.75)
            //     .andThen(elevator.stepToCommand(Position.BARGE))
            // ))
            // .andThen(new ParallelCommandGroup(
            //     drive.followPathCommand("BackUpFromBarge", false),
            //     sleep(0.5)
            //     .andThen(elevator.stepToCommand(Position.LOAD))
            // ))
        )
    );

    private int commandIdx = 0;

    private String[] commandNames;
    private String allCommands;

    public static Auton getInstance() {
        if (mInstance == null) mInstance = new Auton();
        return mInstance;
    }

    private Auton() {
        commandNames = commands.keySet().toArray(new String[0]);
        Arrays.sort(commandNames);
        allCommands = String.join("\n", commandNames);
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
