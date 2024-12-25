package org.firstinspires.ftc.teamcode;

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
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utilities.constants.Constants;

@TeleOp
public class BlueTeleop extends OpMode {
    private MecanumDrive drivetrain;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private DcMotor viperMotor;
    private Gamepad driverController, operatorController;
    YawPitchRollAngles robotOrientation;
    private BHI260IMU imu;
    IMU.Parameters imuParameters;

    private Servo leftArmServo, rightArmServo, wristServo, grabberServo;
    private Servo linkageServo, leftIntakeArmServo, rightIntakeArmServo, wristIntakeServo, grabberIntakeServo;

    private boolean aButtonPreviousState, bButtonPreviousState, yButtonPreviousState, xButtonPreviousState, leftBumperButtonPreviousState, rightBumperButtonPreviousState;
    private boolean aButtonPreviousStateG2, bButtonPreviousStateG2, yButtonPreviousStateG2, xButtonPreviousStateG2, leftBumperButtonPreviousStateG2, rightBumperButtonPreviousStateG2, DPUPButtonPreviousStateG2;

    @Override
    public void init() {
        drivetrain = new MecanumDrive(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap);

        viperMotor = hardwareMap.get(DcMotorEx.class, "viperMotor");
        viperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        /*

        imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(new Orientation(
                AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                AngleUnit.DEGREES,
                90,
                0,
                0,
                0
        )));

        imu.initialize(imuParameters);

        */

        aButtonPreviousState = false;
        bButtonPreviousState = false;
        yButtonPreviousState = false;
        xButtonPreviousState = false;
        leftBumperButtonPreviousState = false;
        rightBumperButtonPreviousState = false;

        aButtonPreviousStateG2 = false;
        bButtonPreviousStateG2 = false;
        xButtonPreviousStateG2 = false;
        yButtonPreviousStateG2 = false;
        leftBumperButtonPreviousStateG2 = false;
        rightBumperButtonPreviousStateG2 = false;
        DPUPButtonPreviousStateG2 = false;
    }

    @Override
    public void start() {
        grabberServo.setPosition(Constants.outtakeClawClosedPosition);
        leftArmServo.setPosition(Constants.leftouttakeBucketPosition - 0.25);
        rightArmServo.setPosition(Constants.rightouttakeBucketPosition - 0.25);
        wristServo.setPosition(Constants.outtakeWristIntakePosition);

    }

    @Override
    public void loop() {
        //robotOrientation = imu.getRobotYawPitchRollAngles();
        //double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);

        elevator.periodic();

        drivetrain.driveRobotCentric(gamepad1.left_stick_x * 0.75, -gamepad1.left_stick_y * 0.75, gamepad1.right_stick_x * 0.75);
        wristIntakeServo.setPosition(Constants.intakeArmLockPosition);
        if(gamepad1.x && !xButtonPreviousState) { // more values
            grabberServo.setPosition(Constants.outtakeClawOpenPosition);
        } else if (gamepad1.y && !yButtonPreviousState) { // more values
            grabberServo.setPosition(Constants.outtakeClawClosedPosition);
        } else if (gamepad1.b && !bButtonPreviousState) {
            linkageServo.setPosition(Constants.linkageServoInPosition);
            leftIntakeArmServo.setPosition(Constants.leftPivotServoTransferPosition);
            rightIntakeArmServo.setPosition(Constants.rightPivotServoTransferPosition);
        } else if (gamepad1.a && !aButtonPreviousState) {
            linkageServo.setPosition(Constants.linkageServoOutPosition);
            leftIntakeArmServo.setPosition(Constants.leftPivotServoIntakeUpPosition);
            rightIntakeArmServo.setPosition(Constants.rightPivotServoIntakeUpPosition);
        } else if (gamepad1.right_bumper && !rightBumperButtonPreviousState) {
            viperMotor.setTargetPosition(1200);
            viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viperMotor.setPower(Constants.viperConstantVelocity);

            wristServo.setPosition(Constants.outtakeWristScorePosition);
            leftArmServo.setPosition(Constants.leftouttakeArmScorePosition);
            rightArmServo.setPosition(Constants.rightouttakeArmScorePosition);
        } else if (gamepad1.left_bumper && !leftBumperButtonPreviousState) {
            viperMotor.setTargetPosition(2100);
            viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viperMotor.setPower(Constants.viperConstantVelocity);

            wristServo.setPosition(Constants.outtakeWristTransferPosition + 0.06);
        }

        if(gamepad2.right_bumper && !rightBumperButtonPreviousStateG2) {
            rightArmServo.setPosition(Constants.rightouttakeBucketPosition);
            leftArmServo.setPosition(Constants.leftouttakeBucketPosition);
            wristServo.setPosition(Constants.outtakeWristTransferPosition);

            viperMotor.setTargetPosition(4200);
            viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viperMotor.setPower(Constants.viperConstantVelocity);
        } else if(gamepad2.left_bumper && !leftBumperButtonPreviousStateG2) {
            viperMotor.setTargetPosition(0);
            viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viperMotor.setPower(Constants.viperConstantVelocity);

            rightArmServo.setPosition(Constants.rightouttakeArmTransferPosition);
            leftArmServo.setPosition(Constants.rightouttakeArmTransferPosition);
            wristServo.setPosition(Constants.outtakeWristTransferPosition);
            grabberServo.setPosition(Constants.outtakeClawOpenPosition);
        } else if(gamepad2.a && !aButtonPreviousStateG2) {
            grabberIntakeServo.setPosition(Constants.intakeClawOpenPosition);
        } else if(gamepad2.b && !bButtonPreviousStateG2) {
            grabberIntakeServo.setPosition(Constants.intakeClawClosedPosition);
        } else if(gamepad2.y && !yButtonPreviousStateG2) {
            grabberServo.setPosition(Constants.outtakeClawOpenPosition);
            leftArmServo.setPosition(Constants.leftouttakeArmIntakePosition);
            rightArmServo.setPosition(Constants.rightouttakeArmIntakePosition);
            wristServo.setPosition(Constants.outtakeWristIntakePosition);
        } else if (gamepad2.x && !xButtonPreviousStateG2) {
            rightIntakeArmServo.setPosition(Constants.rightPivotServoIntakeDownPosition);
            leftIntakeArmServo.setPosition(Constants.leftPivotServoIntakeDownPosition);
        } else if(gamepad2.dpad_up && !DPUPButtonPreviousStateG2) {
            viperMotor.setTargetPosition(1300);
            viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viperMotor.setPower(Constants.viperConstantVelocity);

            rightArmServo.setPosition(Constants.rightouttakeBucketPosition);
            leftArmServo.setPosition(Constants.leftouttakeBucketPosition);
            wristServo.setPosition(Constants.outtakeWristTransferPosition);
        } else if(gamepad2.dpad_left) {
            grabberServo.setPosition(Constants.outtakeClawClosedPosition);
            grabberIntakeServo.setPosition(Constants.intakeClawOpenPosition);
        } else if(gamepad2.dpad_right) {
            grabberIntakeServo.setPosition(Constants.intakeClawOpenPosition);
            leftIntakeArmServo.setPosition(Constants.leftPivotServoTransferPosition + 0.08);
            rightIntakeArmServo.setPosition(Constants.rightPivotServoTransferPosition + 0.08);
        }

        telemetry.addData("Viper Current Position", viperMotor.getCurrentPosition());

        aButtonPreviousState = gamepad1.a;
        bButtonPreviousState = gamepad1.b;
        xButtonPreviousState = gamepad1.x;
        yButtonPreviousState = gamepad1.y;
        leftBumperButtonPreviousState = gamepad1.left_bumper;
        rightBumperButtonPreviousState = gamepad1.right_bumper;

        aButtonPreviousStateG2 = gamepad2.a;
        bButtonPreviousStateG2 = gamepad2.b;
        xButtonPreviousStateG2 = gamepad2.x;
        yButtonPreviousStateG2 = gamepad2.y;
        leftBumperButtonPreviousStateG2 = gamepad2.left_bumper;
        rightBumperButtonPreviousStateG2 = gamepad2.right_bumper;
        DPUPButtonPreviousStateG2 = gamepad2.dpad_up;

        /*
        if(gamepad1.a && !aButtonPreviousState) {
            viperMotor.setTargetPosition(0);
            viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viperMotor.setPower(0.5);
        } else if(gamepad1.b && !bButtonPreviousState) {
            viperMotor.setTargetPosition(4200);
            viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viperMotor.setPower(0.5);
        } else if(gamepad1.y && !yButtonPreviousState) {
            viperMotor.setTargetPosition(1300);
            viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viperMotor.setPower(0.5);

        } else if(gamepad1.x && !xButtonPreviousState) {
            viperMotor.setTargetPosition(1700);
            viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viperMotor.setPower(0.5);
        }

        if(gamepad1.left_bumper) {
            grabberServo.setPosition(Constants.outtakeClawClosedPosition);
        } else if(gamepad1.right_bumper) {
            grabberServo.setPosition(Constants.outtakeClawOpenPosition);
        }

        if(gamepad2.b) {
            leftIntakeArmServo.setPosition(0.0);
            rightIntakeArmServo.setPosition(0.0);
        } else if (gamepad2.a) {
            leftIntakeArmServo.setPosition(0.68);
            rightIntakeArmServo.setPosition(0.68);
        } else if (gamepad2.x) {
            leftIntakeArmServo.setPosition(0.54);
            rightIntakeArmServo.setPosition(0.54);
            linkageServo.setPosition(0.0);
        } else if (gamepad2.y) {
            leftIntakeArmServo.setPosition(0.17);
            rightIntakeArmServo.setPosition(0.17);
            linkageServo.setPosition(1.0);
        }

        if(gamepad2.dpad_up) {
            leftArmServo.setPosition(0.0);
            rightArmServo.setPosition(0.0);
        }

        if(gamepad2.dpad_down) {
            leftArmServo.setPosition(0.98);
            rightArmServo.setPosition(0.98);
        } else if (gamepad2.dpad_right) {
            wristServo.setPosition(0.0);
        } else if (gamepad2.dpad_left) {
            wristServo.setPosition(1.0);
        }



        telemetry.addData("viper position", viperMotor.getCurrentPosition());

        aButtonPreviousState = gamepad1.a;
        bButtonPreviousState = gamepad1.b;
        xButtonPreviousState = gamepad1.x;
        yButtonPreviousState = gamepad1.y;

        */
    }
}
