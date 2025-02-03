package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class DrivetrainSubsystem extends SubsystemBase {
    private MecanumDrive mecanumDrive;
    private Rev2mDistanceSensor frontDistanceSensor, rearDistanceSensor;

    public DrivetrainSubsystem(HardwareMap aHardwareMap) {
        mecanumDrive = new MecanumDrive(aHardwareMap);
        frontDistanceSensor = aHardwareMap.get(Rev2mDistanceSensor.class, "frontDistanceSensor");
        rearDistanceSensor = aHardwareMap.get(Rev2mDistanceSensor.class, "rearDistanceSensor");
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

    public double getFrontDistance() {
        return frontDistanceSensor.getDistance(DistanceUnit.MM);
    }

    public double getRearDistance() {
        return rearDistanceSensor.getDistance(DistanceUnit.MM);
    }
}
