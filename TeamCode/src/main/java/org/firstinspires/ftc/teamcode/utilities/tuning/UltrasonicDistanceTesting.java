package org.firstinspires.ftc.teamcode.utilities.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@TeleOp
public class UltrasonicDistanceTesting extends OpMode {
    private DrivetrainSubsystem drivetrainSubsystem;

    @Override
    public void init() {
        drivetrainSubsystem = new DrivetrainSubsystem(hardwareMap, telemetry);
    }

    @Override
    public void start() {
        drivetrainSubsystem.follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        telemetry.addData("Rear Ultrasonic Distance", drivetrainSubsystem.getRearUltrasonicDistance());
    }
}
