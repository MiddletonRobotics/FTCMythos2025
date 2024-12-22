package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    private GamepadEx driverController, operatorController;

    private FtcDashboard dashboard;
    private List<Action> runningActions;

    @Override
    public void init() {
        drivetrain = new DrivetrainSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap);

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        dashboard = FtcDashboard.getInstance();
        runningActions = new ArrayList<>();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        drivetrain.driveRobotCentric(driverController.getLeftX(), driverController.getLeftY(), driverController.getRightX());
    }
}
