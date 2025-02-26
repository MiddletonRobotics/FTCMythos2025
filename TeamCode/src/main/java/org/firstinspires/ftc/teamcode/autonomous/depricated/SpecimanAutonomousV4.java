package org.firstinspires.ftc.teamcode.autonomous.depricated;

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

@Autonomous(name="4SpecAuto", group = "Working")
public class SpecimanAutonomousV4 extends CommandOpMode {
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
                        Commands.fastPath(follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        Commands.sleep(100).andThen(Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(1)).alongWith(Commands.retractElevator(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(2)).alongWith(Commands.intakeFromWall(elevatorSubsystem)),
                        Commands.sleep(300).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(3)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        Commands.sleep(100).andThen(Commands.scoreSpecimanThenRetract(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(4)),
                        Commands.fastPath(follower, chain.getPath(5)).alongWith(Commands.intakeFromWall(elevatorSubsystem)),
                        Commands.sleep(300).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(6)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        Commands.sleep(100).andThen(Commands.scoreSpecimanThenRetract(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(7)),
                        Commands.fastPath(follower, chain.getPath(8)).alongWith(Commands.intakeFromWall(elevatorSubsystem)),
                        Commands.sleep(300).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(9)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        Commands.sleep(100).andThen(Commands.scoreSpecimanThenRetract(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(10))
                )
        );
    }
}
