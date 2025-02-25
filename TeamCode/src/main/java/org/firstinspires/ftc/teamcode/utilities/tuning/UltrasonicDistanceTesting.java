package org.firstinspires.ftc.teamcode.utilities.tuning;

import com.arcrobotics.ftclib.command.CommandScheduler;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utilities.PIDFController;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@TeleOp
public class UltrasonicDistanceTesting extends OpMode {
    private DrivetrainSubsystem drivetrainSubsystem;
    private org.firstinspires.ftc.teamcode.utilities.PIDFController pidController;

    public static double target = 0.8;
    private double lastErrorDistance, errorDistance, distance = 0;
    private double correctionY;

    private static final double kPtx = 0.155;
    private static final double kDtx = 0.03;

    @Override
    public void init() {
        drivetrainSubsystem = new DrivetrainSubsystem(hardwareMap, telemetry);
        pidController = new PIDFController(0.145, 0.0, 0.03, 0.0);
    }

    @Override
    public void start() {
        drivetrainSubsystem.follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        drivetrainSubsystem.follower.update();
        distance = drivetrainSubsystem.getRearUltrasonicDistance();
        errorDistance = distance - target;

        double derivativeDistance = errorDistance - lastErrorDistance;
        lastErrorDistance = errorDistance;

        correctionY = (kPtx * errorDistance) + (kDtx * derivativeDistance);

        drivetrainSubsystem.driveRobotPedroFollower(0, -correctionY, 0);
        telemetry.addData("Distance from closest wall: ", distance);
        telemetry.addData("Error Distance", errorDistance);
        telemetry.addData("CorrectionY", correctionY);
    }
}
