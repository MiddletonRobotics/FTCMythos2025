package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;

public class DrivetrainSubsystem extends SubsystemBase {
    private MecanumDrive mecanumDrive;
    private AnalogInput rearUltra;
    private Telemetry telemetry;

    public Follower follower;
    private PathBuilder builder;
    private boolean breakFollowingSupplier;

    public enum AutoScoringState {
        ON,
        OFF
    }

    private AutoScoringState autoScoringState = AutoScoringState.OFF;

    public DrivetrainSubsystem(HardwareMap aHardwareMap, Telemetry telemetry) {
        mecanumDrive = new MecanumDrive(aHardwareMap);
        rearUltra = aHardwareMap.get(AnalogInput.class, "rearUltra");

        follower = new Follower(aHardwareMap);
        follower.setPose(new Pose(0.5, 66.0, 0.0));
        this.telemetry = telemetry;
    }

    public PathChain getAutoScoringPath() {
        builder = new PathBuilder();
        return builder
            .addPath(
                new BezierLine(
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN),
                        new Point(31.000, 80.000, Point.CARTESIAN)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(0))
            .addPath(
                new BezierLine(
                        new Point(31.000, 80.000, Point.CARTESIAN),
                        new Point(follower.getPose().getX(), follower.getPose().getY(), Point.CARTESIAN)
                )
            ).setConstantHeadingInterpolation(Math.toRadians(0)).build();
    }

    public double getRearUltrasonicDistance() {
        if (rearUltra == null) {
            telemetry.addData("Warning", "Ultrasonic sensor not initialized");
            return Double.MAX_VALUE;
        }
        return (100 * (rearUltra.getVoltage() / 5)) / 2.54;
    }

    public void setAutoScoringState(AutoScoringState autoScoringState) {
        this.autoScoringState = autoScoringState;
    }

    public AutoScoringState getAutoScoringState() {
        return this.autoScoringState;
    }

    @Override
    public void periodic() {
        /*
        telemetry.addData("Drivetrain Pose X", follower.getPose().getX());
        telemetry.addData("Drivetrain Pose Y", follower.getPose().getY());
        telemetry.addData("Drivetrain Heading", follower.getPose().getHeading());
        telemetry.update();

         */

        /*

        switch (autoScoringState) {
            case ON:
                breakFollowingSupplier = false;
            case OFF:
                if(!breakFollowingSupplier) {
                    follower.breakFollowing();
                    follower.startTeleopDrive();
                    breakFollowingSupplier = true;
                }

                break;
        }


         */

        follower.update();
    }

    public void resetHeading(IMU imu) {
        imu.resetYaw();
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double rotationalSpeed, IMU imu, boolean squareInputs) {
        YawPitchRollAngles gyroAngles = imu.getRobotYawPitchRollAngles();
        mecanumDrive.driveFieldCentric(strafeSpeed, forwardSpeed, rotationalSpeed, gyroAngles.getYaw(AngleUnit.DEGREES), squareInputs);
    }

    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double rotationalSpeed, IMU imu) {
        driveFieldCentric(strafeSpeed, forwardSpeed, rotationalSpeed, imu, false);
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double rotationalSpeed, boolean squareInputs) {
        mecanumDrive.driveRobotCentric(strafeSpeed, forwardSpeed, rotationalSpeed, squareInputs);
    }

    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double rotationalSpeed) {
        driveRobotCentric(strafeSpeed, forwardSpeed, rotationalSpeed, false);
    }

    public void driveRobotPedroFollower(double strafeSpeed, double forwardSpeed, double rotationalSpeed) {
        follower.setTeleOpMovementVectors(forwardSpeed, -strafeSpeed, -rotationalSpeed);
    }
}
