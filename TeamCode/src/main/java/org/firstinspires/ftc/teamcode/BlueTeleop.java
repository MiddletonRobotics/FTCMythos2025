package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.MecanumDrive;

@TeleOp
public class BlueTeleop extends OpMode {
    private Motor frontLeft, frontRight, backLeft, backRight;
    private MecanumDrive drivetrain;
    private GamepadEx driverController;
    private RevIMU imu;
    private static boolean isRobotRelative = true;

    @Override
    public void init() {
        frontLeft = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        frontRight = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        backLeft = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);
        backRight = new Motor(hardwareMap, "frontLeft", Motor.GoBILDA.RPM_312);

        imu = new RevIMU(hardwareMap);
        imu.init();

        drivetrain = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
        driverController = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        if(isRobotRelative){
            drivetrain.driveRobotCentric(driverController.getLeftX(), driverController.getLeftY(), driverController.getRightY(), false);
        } else {
            drivetrain.driveFieldCentric(
                    driverController.getLeftX(),
                    driverController.getLeftY(),
                    driverController.getRightY(),
                    imu.getRotation2d().getDegrees(),
                    false
            );
        }
    }
}
