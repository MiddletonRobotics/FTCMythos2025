package org.firstinspires.ftc.teamcode.utilities.tuning;

import com.arcrobotics.ftclib.command.CommandScheduler;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@TeleOp
public class UltrasonicDistanceTesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        DrivetrainSubsystem drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry);

        waitForStart();

        while (!isStopRequested()) {
            telemetry.addData("left sensor", drivetrain.getRearUltrasonicDistance());
            telemetry.update();

            CommandScheduler.getInstance().run();
        }
    }
}
