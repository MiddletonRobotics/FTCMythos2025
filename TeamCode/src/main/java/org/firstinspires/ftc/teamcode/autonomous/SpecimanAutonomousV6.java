package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Autonomous(name="W-5SpecNoPreload", group = "Working")
public class SpecimanAutonomousV6 extends CommandOpMode {
    public PathChain chain;
    public Follower follower;

    private ElevatorSubsystem elevatorSubsystem;
    private IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        this.elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry);
        this.intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(new Pose(0.5, 73.0, 0.0));
        this.chain = Paths.fiveSpecimanAutoRenewed;

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(

                )
        );
    }
}