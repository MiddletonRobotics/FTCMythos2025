package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.paths.Paths;
import org.firstinspires.ftc.teamcode.commands.Commands;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

@Autonomous(name="Auto???", group = "WIP")
public class SpecimanAutonomousV2 extends CommandOpMode {
    public PathChain chain;
    public Follower follower;

    private ElevatorSubsystem elevatorSubsystem;

    @Override
    public void initialize() {
        this.elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry);
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(new Pose(0.5, 73.0, 0.0));
        this.chain = Paths.fiveSpecimanAuto;

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        Commands.followPath(follower, chain.getPath(0)).alongWith(
                                Commands.prepareSpeciman(elevatorSubsystem)
                        ),
                        Commands.sleep(150).andThen(
                                Commands.scoreSpecimanThenRetract(elevatorSubsystem)
                        ),
                        Commands.followPath(follower, chain.getPath(1)),
                        Commands.followPath(follower, chain.getPath(2)).alongWith(Commands.intakeFromWall(elevatorSubsystem)),
                        Commands.followPath(follower, chain.getPath(3)),
                        Commands.sleep(300).andThen(Commands.closeClaw(elevatorSubsystem)).andThen(Commands.sleep(150)),
                        Commands.followPath(follower, chain.getPath(4)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        Commands.sleep(100).andThen(
                                Commands.scoreSpecimanThenRetract(elevatorSubsystem)
                        ).alongWith(Commands.sleep(1200).andThen(Commands.followPath(follower, chain.getPath(5)))),
                        Commands.followPath(follower, chain.getPath(6)).alongWith(Commands.intakeFromWall(elevatorSubsystem)),
                        Commands.followPath(follower, chain.getPath(7)),
                        Commands.sleep(300).andThen(Commands.closeClaw(elevatorSubsystem)).andThen(Commands.sleep(150)),
                        Commands.followPath(follower, chain.getPath(8)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        Commands.sleep(100).andThen(
                                Commands.scoreSpecimanThenRetract(elevatorSubsystem)
                        ),
                        Commands.followPath(follower, chain.getPath(9)),
                        Commands.followPath(follower, chain.getPath(10)).alongWith(Commands.intakeFromWall(elevatorSubsystem)),
                        Commands.followPath(follower, chain.getPath(11)),
                        Commands.sleep(300).andThen(Commands.closeClaw(elevatorSubsystem)).andThen(Commands.sleep(150)),
                        Commands.followPath(follower, chain.getPath(12)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        Commands.sleep(100).andThen(
                                Commands.scoreSpecimanThenRetract(elevatorSubsystem)
                        ),
                        Commands.intakeFromWall(elevatorSubsystem),
                        Commands.followPath(follower, chain.getPath(13)),
                        Commands.sleep(300).andThen(Commands.closeClaw(elevatorSubsystem)).andThen(Commands.sleep(150)),
                        Commands.followPath(follower, chain.getPath(14)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        Commands.sleep(100).andThen(
                                Commands.scoreSpecimanThenRetract(elevatorSubsystem),
                        Commands.followPath(follower, chain.getPath(15))
                        )
                )
        );
    }
}
