package org.firstinspires.ftc.teamcode.autonomous.paths;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class FiveSpecimanAuto {
    public static PathChain path() {
            PathBuilder builder = new PathBuilder();

            builder
                    .addPath(
                            // Line 1
                            new BezierLine(
                                    new Point(0.500, 73.000, Point.CARTESIAN),
                                    new Point(33.000, 83.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 2
                            new BezierCurve(
                                    new Point(33.000, 83.000, Point.CARTESIAN),
                                    new Point(1.000, 38.000, Point.CARTESIAN),
                                    new Point(57.000, 45.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 3
                            new BezierCurve(
                                    new Point(57.000, 45.000, Point.CARTESIAN),
                                    new Point(68.000, 39.000, Point.CARTESIAN),
                                    new Point(5.000, 36.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 4
                            new BezierLine(
                                    new Point(5.000, 36.000, Point.CARTESIAN),
                                    new Point(31.500, 81.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 5
                            new BezierCurve(
                                    new Point(31.500, 81.000, Point.CARTESIAN),
                                    new Point(1.000, 39.000, Point.CARTESIAN),
                                    new Point(58.000, 34.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 6
                            new BezierCurve(
                                    new Point(58.000, 34.000, Point.CARTESIAN),
                                    new Point(68.000, 27.000, Point.CARTESIAN),
                                    new Point(4.000, 27.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 7
                            new BezierLine(
                                    new Point(4.000, 27.000, Point.CARTESIAN),
                                    new Point(31.500, 80.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 8
                            new BezierCurve(
                                    new Point(31.500, 80.000, Point.CARTESIAN),
                                    new Point(1.000, 39.000, Point.CARTESIAN),
                                    new Point(60.000, 23.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 9
                            new BezierCurve(
                                    new Point(60.000, 23.000, Point.CARTESIAN),
                                    new Point(68.000, 18.000, Point.CARTESIAN),
                                    new Point(4.000, 23.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 10
                            new BezierCurve(
                                    new Point(4.000, 23.000, Point.CARTESIAN),
                                    new Point(7.000, 52.000, Point.CARTESIAN),
                                    new Point(31.500, 79.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 11
                            new BezierLine(
                                    new Point(31.500, 79.000, Point.CARTESIAN),
                                    new Point(4.000, 36.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 12
                            new BezierLine(
                                    new Point(4.000, 36.000, Point.CARTESIAN),
                                    new Point(31.500, 77.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .addPath(
                            // Line 13
                            new BezierLine(
                                    new Point(31.500, 78.000, Point.CARTESIAN),
                                    new Point(4.000, 36.000, Point.CARTESIAN)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0));

        return builder.build();
    }
}
