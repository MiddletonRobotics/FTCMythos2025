package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.LinkedList;
import java.util.Queue;

public class DrivetrainSubsystem extends SubsystemBase {
    private MecanumDrive mecanumDrive;
    private AnalogInput ultraSensorLeft, ultraSensorRear;

    private DistanceUnit unit = DistanceUnit.INCH;

    private final Queue<Double> sensorLeftData = new LinkedList<>();
    private final Queue<Double> sensorRearData = new LinkedList<>();

    private double sensorLeftAverage;
    private double sensorRearAverage;

    public static int rollingAverageSize = 3;

    public DrivetrainSubsystem(HardwareMap aHardwareMap) {
        mecanumDrive = new MecanumDrive(aHardwareMap);
        ultraSensorLeft = aHardwareMap.get(AnalogInput.class, "leftUltraSensor");
        ultraSensorRear = aHardwareMap.get(AnalogInput.class, "rearUltraSensor");
    }

    @Override
    public void periodic() {
        sensorLeftData.add(ultraSensorLeft.getVoltage());
        if (sensorLeftData.size() > rollingAverageSize) {
            sensorLeftData.remove();
        }

        // noinspection OptionalGetWithoutIsPresent
        sensorLeftAverage = sensorLeftData.stream()
                .reduce((total, el) -> total + el / sensorLeftData.size()).get();

        sensorRearData.add(ultraSensorRear.getVoltage());
        if (sensorRearData.size() > rollingAverageSize) {
            sensorRearData.remove();
        }

        // noinspection OptionalGetWithoutIsPresent
        sensorRearAverage = sensorRearData.stream()
                .reduce((total, el) -> total + el / sensorRearData.size()).get();
    }

    public void setDistanceUnit(DistanceUnit distanceUnit) {
        unit = distanceUnit;
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

    public double getSensorLeft() {
        return unit.fromCm(sensorLeftAverage * 500 / 3.3);
    }

    public double getSensorRight() {
        return unit.fromCm(sensorRearAverage * 500 / 3.3);
    }
}
