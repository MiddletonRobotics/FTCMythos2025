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
        this.chain = Paths.fiveSpecimanNoPreload;

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        Commands.intakeIn(intakeSubsystem),
                        Commands.fastPath(follower, chain.getPath(0)),
                        Commands.fastPath(follower, chain.getPath(1)),
                        Commands.fastPath(follower, chain.getPath(2)),
                        Commands.fastPath(follower, chain.getPath(3)),
                        Commands.fastPath(follower, chain.getPath(4)),
                        Commands.fastPath(follower, chain.getPath(5)).alongWith(Commands.intakeFromWall(elevatorSubsystem)),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(6)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(7)).alongWith(Commands.retractThenIntake(elevatorSubsystem)),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(8)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(9)).alongWith(Commands.retractThenIntake(elevatorSubsystem)),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(10)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(11)).alongWith(Commands.retractThenIntake(elevatorSubsystem)),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(12)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(13)).alongWith(Commands.retractThenIntake(elevatorSubsystem)),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(14)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(15)).alongWith(Commands.retractElevator(elevatorSubsystem))
                )
        );
    }
    @Override
    public void runOpMode() {
        initialize();

        while (!opModeIsActive()) {
            elevatorSubsystem.onAutoInit();
            intakeSubsystem.onInit();
        }

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
            telemetry.addData("Viper Motor Power", elevatorSubsystem.viperMotor.getPower());
            telemetry.addData("Follower Position X", follower.getPose().getX());
            telemetry.addData("Follower Position Y", follower.getPose().getX());
            telemetry.addData("Follower Heading", follower.getPose().getHeading());
            telemetry.addData("Follower Busy", follower.isBusy());
        }

        reset();
    }
}