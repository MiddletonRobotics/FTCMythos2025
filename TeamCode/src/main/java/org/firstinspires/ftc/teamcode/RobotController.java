package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.commands.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.commands.PrepareSpeciman;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="RobotController")
public class RobotController extends CommandOpMode {
    private DrivetrainSubsystem drivetrain;
    private IntakeSubsystem intake;
    private ElevatorSubsystem elevator;
    private IMU imu;

    private GamepadEx driverController, operatorController;
    private GamepadButton outtakeClaw, intakeClaw, retractViper, prepareSpeciman, scoreSpeciman;

    private FtcDashboard dashboard;
    private List<Action> runningActions;

    @Override
    public void initialize() {
        drivetrain = new DrivetrainSubsystem(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        outtakeClaw = new GamepadButton(driverController, GamepadKeys.Button.X);
        retractViper = new GamepadButton(driverController, GamepadKeys.Button.B);
        prepareSpeciman = new GamepadButton(driverController, GamepadKeys.Button.LEFT_BUMPER);
        scoreSpeciman = new GamepadButton(driverController, GamepadKeys.Button.RIGHT_BUMPER);
        intakeClaw = new GamepadButton(operatorController, GamepadKeys.Button.A);

        outtakeClaw
                .whenPressed(new InstantCommand((() -> elevator.setClawState(ElevatorSubsystem.ClawState.OPEN_CLAW)), elevator))
                .whenReleased(new InstantCommand((() -> elevator.setClawState(ElevatorSubsystem.ClawState.CLOSE_CLAW)), elevator));

        retractViper.whenPressed(new InstantCommand((() -> elevator.setLiftState(ElevatorSubsystem.LiftState.RETRACTED)), elevator));
        prepareSpeciman.whenPressed(new PrepareSpeciman(elevator));
        scoreSpeciman.whenPressed(new InstantCommand((() -> elevator.setLiftState(ElevatorSubsystem.LiftState.SPECIMAN_SCORE)), elevator));

        register(drivetrain, elevator);
        schedule(new RunCommand(telemetry::update));
        drivetrain.setDefaultCommand(new FieldOrientedDrive(drivetrain, driverController::getLeftX, driverController::getLeftY, driverController::getRightX, imu));
    }
}
