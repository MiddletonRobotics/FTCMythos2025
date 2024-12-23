package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="RobotController")
public class RobotController extends OpMode {
    private DrivetrainSubsystem drivetrain;
    private IntakeSubsystem intake;
    private ElevatorSubsystem elevator;
    private IMU imu;

    private GamepadEx driverController, operatorController;

    private boolean openIntakeClaw;
    private boolean openOuttakeClaw;
    private boolean closeIntakeClaw;
    private boolean closeOuttakeClaw;

    private FtcDashboard dashboard;
    private List<Action> runningActions;

    @Override
    public void init() {
        drivetrain = new DrivetrainSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        openIntakeClaw = operatorController.wasJustPressed(GamepadKeys.Button.A);
        openOuttakeClaw = driverController.wasJustPressed(GamepadKeys.Button.X);
        closeIntakeClaw = operatorController.wasJustReleased(GamepadKeys.Button.B);
        closeOuttakeClaw = driverController.wasJustReleased(GamepadKeys.Button.Y);

        dashboard = FtcDashboard.getInstance();
        runningActions = new ArrayList<>();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        if(openOuttakeClaw) {
            elevator.openClaw();
        } else if (closeOuttakeClaw) {
            elevator.closeClaw();
        }

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        drivetrain.driveFieldCentric(driverController.getLeftX(), driverController.getLeftY(), driverController.getRightX(), orientation.getYaw(AngleUnit.DEGREES));
    }
}
