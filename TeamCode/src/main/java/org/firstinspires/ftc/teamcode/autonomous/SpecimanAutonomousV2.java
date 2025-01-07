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
                        Commands.sleep(500).andThen(Commands.closeClaw(elevatorSubsystem)).andThen(Commands.sleep(200)),
                        Commands.followPath(follower, chain.getPath(4)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        Commands.sleep(150).andThen(
                                Commands.scoreSpecimanThenRetract(elevatorSubsystem)
                        ),
                        Commands.followPath(follower, chain.getPath(5)),
                        Commands.followPath(follower, chain.getPath(6))
                )
        );
    }
}
