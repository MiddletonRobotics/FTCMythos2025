package org.firstinspires.ftc.teamcode.autonomous.paths;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

public class FiveSpecimanAutoRenewed {
    public static PathChain path() {
        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(0.500, 73.000, Point.CARTESIAN),
                                new Point(30.000, 90.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(30.000, 90.000, Point.CARTESIAN),
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
                                new Point(-1.000, 21.500, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 8
                        new BezierLine(
                                new Point(-1.000, 22.000, Point.CARTESIAN),
                                new Point(31.000, 85.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(31.000, 85.000, Point.CARTESIAN),
                                new Point(2.500, 36.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(2.500, 36.000, Point.CARTESIAN),
                                new Point(31.000, 84.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(31.000, 84.000, Point.CARTESIAN),
                                new Point(2.500, 36.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(2.500, 36.000, Point.CARTESIAN),
                                new Point(31.000, 82.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(31.000, 82.000, Point.CARTESIAN),
                                new Point(2.500, 36.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 14
                        new BezierCurve(
                                new Point(2.500, 36.000, Point.CARTESIAN),
                                new Point(15.000, 70.000, Point.CARTESIAN),
                                new Point(31.000, 81.000, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0));
        
        return builder.build();
    }
}
