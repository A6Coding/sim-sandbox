package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.util.PathPlannerUtils;

import java.util.Optional;

public class AutoCommands {
    private final CommandSwerveDrivetrain drivetrain;
    private final ArmSubsystem arm;
    private final ElevatorSubsystem elevator;
    private final GrabberSubsystem grabber;

    public AutoCommands(CommandSwerveDrivetrain drivetrain, ArmSubsystem arm,
                        ElevatorSubsystem elevator, GrabberSubsystem grabber)
    {
        this.drivetrain = drivetrain;
        this.arm = arm;
        this.elevator = elevator;
        this.grabber = grabber;
    }

    public Command one() {
        return elevator.autoOne().alongWith(arm.autoOne()).alongWith(Commands.print("running one"));
    }

    public Command two() {
        return elevator.autoTwo().alongWith(arm.autoTwo()).alongWith(Commands.print("running one"));
    }

    public Command three() {
        return elevator.autoThree().alongWith(arm.autoThree()).alongWith(Commands.print("running one"));
    }

    public Command four() {
        return elevator.autoFour().alongWith(arm.autoFour()).alongWith(Commands.print("running one"));
    }

    public Command TwoBallAuto() {
        Optional<PathPlannerPath> pathOne = PathPlannerUtils.loadPathByName("PathOne");
        Optional<PathPlannerPath> pathTwo = PathPlannerUtils.loadPathByName("PathTwo");
        Optional<PathPlannerPath> pathThree = PathPlannerUtils.loadPathByName("PathThree");
        Optional<PathPlannerPath> pathFour = PathPlannerUtils.loadPathByName("PathFour");

        PathPlannerAuto auto;

        System.out.println("has paths: " + (!pathOne.isEmpty()));

        var cmd = pathOne.isEmpty()
                ? Commands.none()
                : Commands.sequence(
                        one(),
                        Commands.waitSeconds(2.5),
                        (AutoBuilder.followPath(pathOne.get())
                                .alongWith(Commands.print("Running Path One"))),
                        two(),
                        Commands.waitSeconds(2.5),
                        (AutoBuilder.followPath(pathTwo.get())
                                .alongWith(Commands.print("Running Path Two"))),
                        three(),
                        Commands.waitSeconds(2.5),
                        (AutoBuilder.followPath(pathThree.get())
                                .alongWith(Commands.print("Running Path Two"))),
                        four(),
                        Commands.waitSeconds(2.5),
                        (AutoBuilder.followPath(pathFour.get())
                                .alongWith(Commands.print("Running Path Two")))
                );


        auto = new PathPlannerAuto(cmd);

        return auto;

    }

    public SendableChooser<Command> buildAutoCommandChooser() {
        SendableChooser<Command> auto = new SendableChooser<>();
        auto.addOption("Two Ball Auto", TwoBallAuto());
        return auto;
    }

}


