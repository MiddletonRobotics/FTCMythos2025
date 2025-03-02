package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
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
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Autonomous(name="W-5BucketGolden", group = "Working")
public class BucketAutonomousGolden extends CommandOpMode {
    public PathChain chain;
    public Follower follower;

    private ElevatorSubsystem elevatorSubsystem;
    private IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        this.elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry);
        this.intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(new Pose(0.5, 110.0, 0.0));
        this.chain = Paths.fourBucketAuto;

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        Commands.intakeIn(intakeSubsystem).alongWith(Commands.followPath(follower, chain.getPath(0)).alongWith(Commands.prepareBucket(elevatorSubsystem))),
                        Commands.sleep(300),
                        Commands.scoreBucketThenRetract(elevatorSubsystem),
                        Commands.prepareIntakeWithoutExtension(elevatorSubsystem, intakeSubsystem).alongWith(Commands.followPath(follower, chain.getPath(1))),
                        Commands.sleep(100),
                        Commands.IntakeDown(intakeSubsystem),
                        Commands.sleep(100),
                        Commands.sleep(300).andThen(Commands.intakeTransferWithoutRetraction(elevatorSubsystem, intakeSubsystem)),
                        Commands.followPath(follower, chain.getPath(2)).alongWith(Commands.prepareBucket(elevatorSubsystem)),
                        Commands.sleep(300),
                        Commands.scoreBucketThenRetract(elevatorSubsystem),
                        Commands.followPath(follower, chain.getPath(3)).alongWith(Commands.prepareIntakeWithoutExtension(elevatorSubsystem, intakeSubsystem)),
                        Commands.sleep(100),
                        Commands.IntakeDown(intakeSubsystem),
                        Commands.sleep(100),
                        Commands.sleep(300).andThen(Commands.intakeTransferWithoutRetraction(elevatorSubsystem, intakeSubsystem)),
                        Commands.followPath(follower, chain.getPath(4)).alongWith(Commands.prepareBucket(elevatorSubsystem)),
                        Commands.sleep(300),
                        Commands.scoreBucketThenRetract(elevatorSubsystem),
                        Commands.prepareIntakeWithoutExtension(elevatorSubsystem, intakeSubsystem).alongWith(Commands.followPath(follower, chain.getPath(5))),
                        Commands.RotateIntakeToPosition(intakeSubsystem),
                        Commands.sleep(100),
                        Commands.IntakeDown(intakeSubsystem),
                        Commands.sleep(100),
                        Commands.sleep(300).andThen(Commands.intakeTransferWithoutRetraction(elevatorSubsystem, intakeSubsystem)),
                        Commands.followPath(follower, chain.getPath(6)).alongWith(Commands.prepareBucket(elevatorSubsystem)),
                        Commands.sleep(300).andThen(Commands.scoreBucketThenRetract(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(9))
                )
        );
    }

    @Override
    public void runOpMode() {
        initialize();

        while (!opModeIsActive()) {
            elevatorSubsystem.onAutoInit();
        }

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
            /*
            telemetry.addData("Viper Motor Power", elevatorSubsystem.viperMotor.getPower());
            telemetry.addData("Follower Position X", follower.getPose().getX());
            telemetry.addData("Follower Position Y", follower.getPose().getX());
            telemetry.addData("Follower Heading", follower.getPose().getHeading());
            telemetry.addData("Follower Busy", follower.isBusy());

             */
        }

        reset();
    }
}
