package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;

import java.util.LinkedList;
import java.util.Queue;

public class DrivetrainSubsystem extends SubsystemBase {
    private MecanumDrive mecanumDrive;
    private AnalogInput rearUltra;
    private Telemetry telemetry;

    public Follower follower;
    public Pose pickupPosition = new Pose(0,0,0);

    private DistanceUnit unit = DistanceUnit.INCH;
    private final Queue<Double> sensorRearData = new LinkedList<>();
    private double sensorRearAverage;

    public static int rollingAverageSize = 3;

    public DrivetrainSubsystem(HardwareMap aHardwareMap, Telemetry telemetry) {
        mecanumDrive = new MecanumDrive(aHardwareMap);
        rearUltra = aHardwareMap.get(AnalogInput.class, "rearUltra");

        follower = new Follower(aHardwareMap);
        follower.setPose(new Pose(0.5, 73.0, 0.0));
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        sensorRearData.add(rearUltra.getVoltage());
        if (sensorRearData.size() > rollingAverageSize) {
            sensorRearData.remove();
        }

        // noinspection OptionalGetWithoutIsPresent
        sensorRearAverage = sensorRearData.stream().reduce((total, el) -> total + el / sensorRearData.size()).get();

        telemetry.addData("Drivetrain Pose X", follower.getPose().getX());
        telemetry.addData("Drivetrain Pose Y", follower.getPose().getY());
        telemetry.addData("Drivetrain Heading", follower.getPose().getHeading());
        telemetry.addData("Rear Ultrasonic Reading", sensorRearAverage);
        telemetry.update();
    }

    public void setDistanceUnit(DistanceUnit distanceUnit) {
        unit = distanceUnit;
    }

    public double getRearUltrasonicDistance() {
        return unit.fromCm(sensorRearAverage * 500 / 3.3);
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
