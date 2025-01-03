package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@TeleOp(name="TuningMode")
public class TuningTeleop extends OpMode {
    private MecanumDrive drivetrain;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private DcMotorEx viperMotor;
    private Gamepad driverController, operatorController;
    YawPitchRollAngles robotOrientation;
    private BHI260IMU imu;
    IMU.Parameters imuParameters;

    private Servo leftArmServo, rightArmServo, wristServo, grabberServo;
    private Servo linkageServo, leftIntakeArmServo, rightIntakeArmServo, wristIntakeServo, grabberIntakeServo;

    private PIDController elevatorController;

    @Override
    public void init() {
        drivetrain = new MecanumDrive(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);

        viperMotor = hardwareMap.get(DcMotorEx.class, "viperMotor");
        viperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grabberServo = hardwareMap.get(Servo.class, "outtakeClaw");
        wristServo = hardwareMap.get(Servo.class, "outtakeWrist");
        leftArmServo = hardwareMap.get(Servo.class, "leftouttakeArm");
        rightArmServo = hardwareMap.get(Servo.class, "rightouttakeArm");

        grabberServo.setDirection(Servo.Direction.REVERSE);
        wristServo.setDirection(Servo.Direction.FORWARD);
        leftArmServo.setDirection(Servo.Direction.REVERSE);
        rightArmServo.setDirection(Servo.Direction.FORWARD);

        linkageServo = hardwareMap.get(Servo.class, "linkageServo");
        leftIntakeArmServo = hardwareMap.get(Servo.class, "leftPivotServo");
        rightIntakeArmServo = hardwareMap.get(Servo.class, "rightPivotServo");
        wristIntakeServo = hardwareMap.get(Servo.class, "intakeArm");
        grabberIntakeServo = hardwareMap.get(Servo.class, "intakeClaw");

        linkageServo.setDirection(Servo.Direction.FORWARD);
        leftIntakeArmServo.setDirection(Servo.Direction.REVERSE);
        rightIntakeArmServo.setDirection(Servo.Direction.FORWARD);
        wristIntakeServo.setDirection(Servo.Direction.FORWARD);
        grabberIntakeServo.setDirection(Servo.Direction.FORWARD);

        elevatorController = new PIDController(0.015, 0, 0.0001);
        elevatorController.setTolerance(50);
    }

    /**
     * Positions for all servos on the IntoTheDeep Robot
     *  - rightElevatorArm and leftElevatorArm is zeroed at the transfer position, where from there
     *    counts up as it rotates through the entire bot
     *  - elevatorWrist is zeroed with the arm all the way down past intaking position with the wrist
     *    facing upward
     */

    @Override
    public void loop() {
        if(gamepad1.a) {
            leftArmServo.setPosition(0.0);
            rightArmServo.setPosition(0.0);
        } else if (gamepad1.b) {
            leftArmServo.setPosition(1.0);
            rightArmServo.setPosition(1.0);
        } else if (gamepad1.x) {
            wristServo.setPosition(0.0);
        } else if (gamepad1.y) {
            wristServo.setPosition(1.0);
        } else if (gamepad1.left_bumper) {
            viperMotor.setPower(elevatorController.calculate(viperMotor.getCurrentPosition(), 3000));
        } else if (gamepad1.right_bumper) {
            viperMotor.setPower(elevatorController.calculate(viperMotor.getCurrentPosition(), 0));
        } else if (gamepad1.dpad_left) {
            leftIntakeArmServo.setPosition(0.0);
            rightIntakeArmServo.setPosition(0.0);
        } else if (gamepad1.dpad_right) {
            leftIntakeArmServo.setPosition(1.0);
            rightIntakeArmServo.setPosition(1.0);
        } else if (gamepad1.dpad_up) {
            linkageServo.setPosition(0.0);
        } else if (gamepad1.dpad_down) {
            linkageServo.setPosition(1.0);
        } else if (gamepad1.left_stick_button) {
            wristIntakeServo.setPosition(0.0);
        } else if (gamepad1.right_stick_button) {
            wristIntakeServo.setPosition(0.66);
        }


        telemetry.addData("TargetPosition", elevatorController.getSetPoint());
        telemetry.addData("CurrentPosition", viperMotor.getCurrentPosition());
        telemetry.addData("CurrentPositionError", elevatorController.getPositionError());
        telemetry.addData("ViperMotorPower", viperMotor.getPower());
    }
}
