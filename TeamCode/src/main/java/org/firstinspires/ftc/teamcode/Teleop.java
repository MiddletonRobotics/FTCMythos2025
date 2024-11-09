package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.constants.Constants;

@TeleOp(name= "Teleop")
public class Teleop extends OpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    private DcMotor ViperMotor;

    private Servo Claw;
    private Servo Arm;

    private boolean aButtonPreviousStateG2, aButtonPreviousState;
    private boolean bButtonPreviousStateG2, bButtonPreviousState;
    private boolean yButtonPreviousStateG2, yButtonPreviousState;
    private boolean xButtonPreviousStateG2, xButtonPreviousState;
    private boolean leftBumperButtonPreviousStateG2, leftBumperButtonPreviousState;
    private boolean rightBumperButtonPreviousStateG2, rightBumperButtonPreviousState;

    private boolean SlowMode;
    private boolean SpeedMode;

    public enum DrivetrainState {
        NORMAL,
        SPEED,
        SLOW
    }

    public enum ArmMode {
            STORED,
            INTAKING,
            OUTTAKING
    }

    public enum ClawMode {
        OPENED,
        CLOSED
    }

    public enum ViperMode {
        RETRACTED,
        CLIMBING,
        SCORING,
        TOPSPECIMAN,
        BOTTOMSPECIMAN,
        TOPGOAL,
        BOTTOMGOAL
    }

    private DrivetrainState DrivetrainMode;
    private ViperMode ViperState;
    private ArmMode ArmState;
    private ClawMode ClawState;

    @Override
    public void init() {
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        ViperMotor = hardwareMap.get(DcMotor.class, "ViperMotor");

        Claw = hardwareMap.get(Servo.class, "Claw");
        Arm = hardwareMap.get(Servo.class, "Arm");

        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        ViperMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Claw.setDirection(Servo.Direction.REVERSE);
        Arm.setDirection(Servo.Direction.REVERSE);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ViperMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        aButtonPreviousStateG2 = false;
        aButtonPreviousState = false;
        bButtonPreviousStateG2 = false;
        xButtonPreviousStateG2 = false;
        yButtonPreviousStateG2 = false;
        leftBumperButtonPreviousState = false;
        rightBumperButtonPreviousState = false;
        bButtonPreviousState = false;
        xButtonPreviousState = false;
        yButtonPreviousState = false;
        rightBumperButtonPreviousStateG2 = false;
        leftBumperButtonPreviousStateG2 = false;

        SlowMode = false;

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop(){};

    @Override
    public void start(){
        resetRuntime();
        Claw.setPosition(Constants.ClawOpenedPosition);
        Arm.setPosition(Constants.ArmStoredPosition);

        DrivetrainMode = DrivetrainState.NORMAL;
        ClawState = ClawMode.OPENED;
        ArmState = ArmMode.STORED;
    };

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;

        telemetry.addData("Controller Left Y", -gamepad1.left_stick_y);
        telemetry.addData("Controller Left X", gamepad1.left_stick_x);
        telemetry.addData("Controller Right X", gamepad1.right_stick_x);

        telemetry.addData("Claw Position", Claw.getPosition());
        telemetry.addData("Arm Position", Arm.getPosition());
        telemetry.addData("Viper Position", ViperMotor.getCurrentPosition());

        telemetry.addData("Arm State", ArmState);
        telemetry.addData("Claw State", ClawState);
        telemetry.addData("Viper State", ViperState);

        if(gamepad2.a && !aButtonPreviousStateG2) {
            if(ArmState == ArmMode.INTAKING || ArmState == ArmMode.OUTTAKING) {
                Claw.setPosition(Constants.ClawClosedPosition);
                ClawState = ClawMode.CLOSED;
            }
        } else if (gamepad2.b && bButtonPreviousStateG2) {
            Claw.setPosition(Constants.ClawOpenedPosition);
            ClawState = ClawMode.OPENED;
        }

        if(gamepad1.a && !aButtonPreviousState) {
            if(ClawState == ClawMode.OPENED) {
                Arm.setPosition(Constants.ArmStoredPosition);
                ArmState = ArmMode.STORED;
            } else {
                Claw.setPosition(Constants.ClawOpenedPosition);
                ClawState = ClawMode.OPENED;
                Arm.setPosition(Constants.ArmStoredPosition);
                ArmState = ArmMode.STORED;
            }
        } else if (gamepad2.x && !xButtonPreviousStateG2) {
            Arm.setPosition(Constants.ArmIntakingPosition);
            ArmState = ArmMode.INTAKING;
        } else if (gamepad2.y && !yButtonPreviousStateG2) {
            Arm.setPosition(Constants.ArmOuttakingPosition);
            ArmState = ArmMode.OUTTAKING;
        }

        if(gamepad1.left_bumper && !leftBumperButtonPreviousState) {
            SlowMode = !SlowMode;
            if(SlowMode) {
                DrivetrainMode = DrivetrainState.SLOW;
            } else {
                DrivetrainMode = DrivetrainState.NORMAL;
            }
        } else if(gamepad1.right_bumper && !rightBumperButtonPreviousState) {
            SpeedMode = !SpeedMode;
            if(SpeedMode) {
                DrivetrainMode = DrivetrainState.SPEED;
            } else {
                DrivetrainMode = DrivetrainState.NORMAL;
            }
        }

        if(gamepad1.b && !bButtonPreviousState) {
            ViperMotor.setTargetPosition(Constants.ViperRetractedPosition);
            ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ViperMotor.setPower(Constants.ViperConstantVelocity);
            ViperState = ViperMode.RETRACTED;
        } else if (gamepad1.x && !xButtonPreviousState){
            ViperMotor.setTargetPosition(Constants.ViperUpperGoalPosition);
            ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ViperMotor.setPower(Constants.ViperConstantVelocity);
            Arm.setPosition(Constants.ArmOuttakingPosition);
            ArmState = ArmMode.OUTTAKING;
            ViperState = ViperMode.TOPGOAL;
        } else if (gamepad1.y && !yButtonPreviousState){
            ViperMotor.setTargetPosition(Constants.ViperLowClimbingPosition);
            ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ViperMotor.setPower(Constants.ViperConstantVelocity);
            Claw.setPosition(Constants.ClawOpenedPosition);
            Arm.setPosition(Constants.ArmStoredPosition);
            ClawState = ClawMode.OPENED;
            ArmState = ArmMode.OUTTAKING;
            ViperState = ViperMode.CLIMBING;
        } else if (gamepad2.right_bumper && !rightBumperButtonPreviousStateG2){
            ViperMotor.setTargetPosition(Constants.ViperUpperSpecimanPosition);
            ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ViperMotor.setPower(Constants.ViperConstantVelocity);
            ViperState = ViperMode.SCORING;
        } else if (gamepad2.left_bumper && !leftBumperButtonPreviousStateG2){
            ViperMotor.setTargetPosition(Constants.ViperUpperSpecimanScoringPosition);
            ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ViperMotor.setPower(Constants.ViperConstantVelocity);
            ViperState = ViperMode.SCORING;
        }

        aButtonPreviousStateG2 = gamepad2.a;
        aButtonPreviousState = gamepad1.a;
        bButtonPreviousStateG2 = gamepad2.b;
        xButtonPreviousStateG2 = gamepad2.x;
        yButtonPreviousStateG2 = gamepad2.y;
        leftBumperButtonPreviousState = gamepad1.left_bumper;
        rightBumperButtonPreviousState = gamepad1.right_bumper;
        bButtonPreviousState = gamepad1.b;
        xButtonPreviousState = gamepad1.x;
        yButtonPreviousState = gamepad1.y;
        rightBumperButtonPreviousStateG2 = gamepad2.right_bumper;
        leftBumperButtonPreviousStateG2 = gamepad2.right_bumper;

        double[] thetaSpeeds = {
                (drive + strafe + rotation),
                (drive - strafe - rotation),
                (drive - strafe + rotation),
                (drive + strafe - rotation),
        };

        if(SlowMode) {
            FrontLeft.setPower(thetaSpeeds[0] * 0.35);
            FrontRight.setPower(thetaSpeeds[1] * 0.35);
            BackLeft.setPower(thetaSpeeds[2] * 0.35);
            BackRight.setPower(thetaSpeeds[3] * 0.35);
        } else if (SpeedMode) {
            FrontLeft.setPower(thetaSpeeds[0]);
            FrontRight.setPower(thetaSpeeds[1]);
            BackLeft.setPower(thetaSpeeds[2]);
            BackRight.setPower(thetaSpeeds[3]);
        } else {
            FrontLeft.setPower(thetaSpeeds[0] * 0.7);
            FrontRight.setPower(thetaSpeeds[1] * 0.7);
            BackLeft.setPower(thetaSpeeds[2] * 0.7);
            BackRight.setPower(thetaSpeeds[3] * 0.7);
        }
    }
}