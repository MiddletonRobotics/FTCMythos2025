package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Autonomous(name="SpecimanAutonomous")
public class SpecimanAutonomous extends OpMode {

    private PathBuilder builder;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private Path scorePreload, park;
    private PathChain pushPickup1, pushPickup2, pushPickup3, scorePickup1, scorePickup2, scorePickup3, scorePickup4;

    private final Pose startPose = new Pose(0.5, 73, Math.toRadians(0));
    private final Pose scorePreloadPose = new Pose(28, 72, Math.toRadians(0));

    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;

    @Override
    public void init() {
        elevator = new ElevatorSubsystem(hardwareMap, telemetry);
        intake =  new IntakeSubsystem(hardwareMap, telemetry);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
    }

    @Override
    public void loop() {
        follower.update();

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        follower.followPath(builder.build());
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(0.500, 73.000, Point.CARTESIAN),
                                new Point(28.000, 82.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(28.000, 82.000, Point.CARTESIAN),
                                new Point(1.000, 38.000, Point.CARTESIAN),
                                new Point(58.500, 46.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierCurve(
                                new Point(58.500, 46.000, Point.CARTESIAN),
                                new Point(68.000, 39.000, Point.CARTESIAN),
                                new Point(15.000, 35.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(15.000, 35.000, Point.CARTESIAN),
                                new Point(117.000, 34.000, Point.CARTESIAN),
                                new Point(15.000, 25.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(15.000, 25.000, Point.CARTESIAN),
                                new Point(117.000, 23.000, Point.CARTESIAN),
                                new Point(15.000, 18.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(15.000, 18.000, Point.CARTESIAN),
                                new Point(7.000, 45.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(7.000, 45.000, Point.CARTESIAN),
                                new Point(31.000, 80.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(31.000, 80.000, Point.CARTESIAN),
                                new Point(7.000, 45.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(7.000, 45.000, Point.CARTESIAN),
                                new Point(31.000, 79.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(31.000, 79.000, Point.CARTESIAN),
                                new Point(7.000, 45.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(7.000, 45.000, Point.CARTESIAN),
                                new Point(31.000, 78.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(31.000, 78.000, Point.CARTESIAN),
                                new Point(7.000, 45.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(7.000, 45.000, Point.CARTESIAN),
                                new Point(31.000, 77.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 14
                        new BezierLine(
                                new Point(31.000, 77.000, Point.CARTESIAN),
                                new Point(7.000, 45.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0));
    }

}
