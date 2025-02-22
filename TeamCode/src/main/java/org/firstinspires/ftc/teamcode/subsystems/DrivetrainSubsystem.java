package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;

public class DrivetrainSubsystem extends SubsystemBase {
    private MecanumDrive mecanumDrive;
    private Telemetry telemetry;
    public Follower follower;

    public DrivetrainSubsystem(HardwareMap aHardwareMap, Telemetry telemetry) {
        mecanumDrive = new MecanumDrive(aHardwareMap);
        follower = new Follower(aHardwareMap);
        follower.setPose(new Pose(0.5, 66.0, 0.0));

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData("Drivetrain Pose X", follower.getPose().getX());
        telemetry.addData("Drivetrain Pose Y", follower.getPose().getY());
        telemetry.addData("Drivetrain Heading", follower.getPose().getHeading());
        telemetry.update();
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
        follower.update();
    }

    public void startPedroPathing() {
        follower.startTeleopDrive();
    }

    public void updateFollower() {
        follower.update();
    }
}
