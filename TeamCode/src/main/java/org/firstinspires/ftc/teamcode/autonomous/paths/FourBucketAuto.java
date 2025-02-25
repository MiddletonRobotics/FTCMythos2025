package org.firstinspires.ftc.teamcode.autonomous.paths;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class FourBucketAuto {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(0.500, 110.000, Point.CARTESIAN),
                                new Point(2.500, 127.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(3.000, 125.500, Point.CARTESIAN),
                                new Point(25.500, 121.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(25.500, 121.000, Point.CARTESIAN),
                                new Point(6.500, 129.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(6.500, 129.000, Point.CARTESIAN),
                                new Point(27.000, 130.100, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(27.000, 130.100, Point.CARTESIAN),
                                new Point(6.000, 128.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(6.000, 128.000, Point.CARTESIAN),
                                new Point(37.000, 129.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(37.000, 129.000, Point.CARTESIAN),
                                new Point(5.500, 128.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(5.500, 128.000, Point.CARTESIAN),
                                new Point(6.000, 106.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(6.000, 106.000, Point.CARTESIAN),
                                new Point(5.500, 128.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(315))
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(5.500, 128.000, Point.CARTESIAN),
                                new Point(6.000, 30.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90));

        return builder.build();
    }
}
