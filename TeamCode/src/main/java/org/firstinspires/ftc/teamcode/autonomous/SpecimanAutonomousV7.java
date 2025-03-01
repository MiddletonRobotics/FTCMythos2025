package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.teamcode.autonomous.paths.FiveSpecimanAutoSensored;
import org.firstinspires.ftc.teamcode.autonomous.paths.Paths;
import org.firstinspires.ftc.teamcode.commands.Commands;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Autonomous(name="W-5SpecSensored", group = "Working")
public class SpecimanAutonomousV7 extends CommandOpMode {
    public PathChain chain;
    public Follower follower;
    public PathBuilder builder;

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

        this.builder = new PathBuilder();

        this.builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(0.500, 73.000, Point.CARTESIAN),
                                new Point(30.500, 85.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(30.500, 85.000, Point.CARTESIAN),
                                new Point(1.000, 38.000, Point.CARTESIAN),
                                new Point(54.000, 41.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(54.000, 41.000, Point.CARTESIAN),
                                new Point(68.000, 39.000, Point.CARTESIAN),
                                new Point(5.000, 36.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(5.000, 36.000, Point.CARTESIAN),
                                new Point(48.000, 33.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(48.000, 33.000, Point.CARTESIAN),
                                new Point(68.000, 27.000, Point.CARTESIAN),
                                new Point(5.000, 27.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(5.000, 27.000, Point.CARTESIAN),
                                new Point(50.000, 23.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(50.000, 23.000, Point.CARTESIAN),
                                new Point(68.000, 18.000, Point.CARTESIAN),
                                new Point(5.000, 21.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(follower.getPose().getX(), 22.000, Point.CARTESIAN),
                                new Point(31.000, 84.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(31.000, 84.000, Point.CARTESIAN),
                                new Point(drivetrainSubsystem.pickupPosition.getX(), 36.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(drivetrainSubsystem.pickupPosition.getX(), 36.000, Point.CARTESIAN),
                                new Point(31.000, 82.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(31.000, 82.000, Point.CARTESIAN),
                                new Point(drivetrainSubsystem.pickupPosition.getX(), 36.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(drivetrainSubsystem.pickupPosition.getX(), 36.000, Point.CARTESIAN),
                                new Point(31.000, 80.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(31.000, 80.000, Point.CARTESIAN),
                                new Point(drivetrainSubsystem.pickupPosition.getX(), 36.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 14
                        new BezierCurve(
                                new Point(drivetrainSubsystem.pickupPosition.getX(), 36.000, Point.CARTESIAN),
                                new Point(15.000, 70.000, Point.CARTESIAN),
                                new Point(31.000, 78.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 15
                        new BezierCurve(
                                new Point(31.000, 78.000, Point.CARTESIAN),
                                new Point(15.000, 70.000, Point.CARTESIAN),
                                new Point(4.000, 36.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0));

        this.chain = builder.build();

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
                        Commands.alignToWall(drivetrainSubsystem, follower).andThen(new InstantCommand(() -> drivetrainSubsystem.pickupPosition = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading()))),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(7)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(8)).alongWith(Commands.retractThenIntake(elevatorSubsystem)),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(9)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(10)).alongWith(Commands.retractThenIntake(elevatorSubsystem)),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(11)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(12)).alongWith(Commands.retractThenIntake(elevatorSubsystem)),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(13)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        Commands.sleep(100),
                        Commands.scoreSpecimanThenRetract(elevatorSubsystem)

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
