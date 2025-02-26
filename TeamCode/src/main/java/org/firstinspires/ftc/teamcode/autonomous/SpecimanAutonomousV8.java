package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.paths.FiveSpecimanAutoSensored;
import org.firstinspires.ftc.teamcode.autonomous.paths.Paths;
import org.firstinspires.ftc.teamcode.commands.Commands;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Autonomous(name="W-5SpecSensored2", group = "Working")
public class SpecimanAutonomousV8 extends CommandOpMode {
    public PathChain chain;
    public Follower follower;

    private DrivetrainSubsystem drivetrainSubsystem;
    private FiveSpecimanAutoSensored path;
    private ElevatorSubsystem elevatorSubsystem;
    private IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        this.drivetrainSubsystem = new DrivetrainSubsystem(hardwareMap, telemetry);
        this.elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry);
        this.intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(new Pose(0.5, 73.0, 0.0));
        this.chain = Paths.fiveSpecimanAutoSensoredFully;

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        Commands.intakeIn(intakeSubsystem),
                        Commands.fastPath(follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        Commands.scoreSpeciman(elevatorSubsystem),
                        Commands.fastPath(follower, chain.getPath(1)).alongWith(Commands.retractElevator(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(2)).alongWith(new InstantCommand(() -> elevatorSubsystem.viperMotor.setMotorDisable())),
                        Commands.fastPath(follower, chain.getPath(3)),
                        Commands.fastPath(follower, chain.getPath(4)),
                        Commands.fastPath(follower, chain.getPath(5)).alongWith(new InstantCommand(() -> elevatorSubsystem.viperMotor.setMotorEnable())),
                        Commands.fastPath(follower, chain.getPath(6)).alongWith(Commands.intakeFromWall(elevatorSubsystem)),
                        Commands.sleep(100),
                        Commands.alignToWall(drivetrainSubsystem, follower),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(7)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(8)).alongWith(Commands.retractThenIntake(elevatorSubsystem)),
                        Commands.sleep(250),
                        Commands.alignToWall(drivetrainSubsystem, follower),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(9)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(10)).alongWith(Commands.retractThenIntake(elevatorSubsystem)),
                        Commands.sleep(250),
                        Commands.alignToWall(drivetrainSubsystem, follower),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(11)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(12)).alongWith(Commands.retractThenIntake(elevatorSubsystem)),
                        Commands.sleep(250),
                        Commands.alignToWall(drivetrainSubsystem, follower),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(13)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        Commands.sleep(100),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(14)).alongWith(Commands.retractElevator(elevatorSubsystem))
                )
        );
    }

    @Override
    public void runOpMode() {
        initialize();

        while (!opModeIsActive()) {
            intakeSubsystem.onInit();
        }

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
            telemetry.addData("Viper Motor Power", elevatorSubsystem.viperMotor.getPower());
            telemetry.addData("Drivetrain Pickup Pose", drivetrainSubsystem.pickupPosition.getX());
            telemetry.addData("Follower Position X", follower.getPose().getX());
            telemetry.addData("Follower Position Y", follower.getPose().getX());
            telemetry.addData("Follower Heading", follower.getPose().getHeading());
            telemetry.addData("Follower Busy", follower.isBusy());
        }

        reset();
    }
}
