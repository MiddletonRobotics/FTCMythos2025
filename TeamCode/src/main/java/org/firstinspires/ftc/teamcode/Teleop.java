package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.constants.Constants;

@TeleOp(name= "Teleop")
public class Teleop extends OpMode {
    private DcMotorEx FrontLeft;
    private DcMotorEx FrontRight;
    private DcMotorEx BackLeft;
    private DcMotorEx BackRight;
    private DcMotorEx ViperMotor;

    private Servo Claw;
    private Servo Arm;

    private boolean aButtonPreviousStateG2, aButtonPreviousState;
    private boolean bButtonPreviousStateG2, bButtonPreviousState;
    private boolean yButtonPreviousStateG2, yButtonPreviousState;
    private boolean xButtonPreviousStateG2, xButtonPreviousState;
    private boolean leftBumperButtonPreviousStateG2;
    private boolean rightBumperButtonPreviousStateG2;
    private boolean dpadUpBumperButtonPreviousState;
    private boolean dpadDownBumperButtonPreviousState;
    private boolean dpadRightBumperButtonPreviousState;
    private boolean dpadLeftBumperButtonPreviousState;

    public enum ArmMode {
            STORED,
            INTAKING,
            DELIVER,
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

    private ViperMode ViperState;
    private ArmMode ArmState;
    private ClawMode ClawState;

    @Override
    public void init() {
        FrontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        ViperMotor = hardwareMap.get(DcMotorEx.class, "ViperMotor");

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
        bButtonPreviousState = false;
        xButtonPreviousState = false;
        yButtonPreviousState = false;
        rightBumperButtonPreviousStateG2 = false;
        leftBumperButtonPreviousStateG2 = false;
        dpadUpBumperButtonPreviousState = false;
        dpadDownBumperButtonPreviousState = false;
        dpadRightBumperButtonPreviousState = false;
        dpadLeftBumperButtonPreviousState = false;

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop(){};

    @Override
    public void start(){
        resetRuntime();
        Claw.setPosition(Constants.ClawOpenedPosition);
        Arm.setPosition(Constants.ArmStoredPosition);

        ClawState = ClawMode.OPENED;
        ArmState = ArmMode.STORED;
    };

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x * 1.1;
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

        if(gamepad1.b && !bButtonPreviousState) {
            ViperMotor.setTargetPosition(Constants.ViperRetractedPosition);
            ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ViperMotor.setPower(Constants.ViperConstantVelocity);
            Arm.setPosition(Constants.ArmStoredPosition);
            Claw.setPosition(Constants.ClawOpenedPosition);
            ClawState = ClawMode.OPENED;
            ArmState = ArmMode.STORED;
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
            Arm.setPosition(Constants.ArmOuttakingPosition);
            Claw.setPosition(Constants.ClawClosedPosition);
            ClawState = ClawMode.CLOSED;
            ArmState = ArmMode.OUTTAKING;
            ViperState = ViperMode.SCORING;
        } else if (gamepad2.left_bumper && !leftBumperButtonPreviousStateG2){
            ViperMotor.setTargetPosition(Constants.ViperUpperSpecimanScoringPosition);
            ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ViperMotor.setPower(Constants.ViperConstantVelocity);
            Arm.setPosition(Constants.ArmDeliverPosition);
            ArmState = ArmMode.DELIVER;
            ViperState = ViperMode.SCORING;
        }

        if(gamepad1.dpad_up) {
            FrontLeft.setPower(0.3);
            FrontRight.setPower(0.3);
            BackLeft.setPower(0.3);
            BackRight.setPower(0.3);
        } else if(gamepad1.dpad_down) {
            FrontLeft.setPower(-0.3);
            FrontRight.setPower(-0.3);
            BackLeft.setPower(-0.3);
            BackRight.setPower(-0.3);
        } else if(gamepad1.dpad_left) {
            FrontLeft.setPower(-0.3);
            FrontRight.setPower(0.3);
            BackLeft.setPower(0.3);
            BackRight.setPower(-0.3);
        } else if(gamepad1.dpad_right) {
            FrontLeft.setPower(0.3);
            FrontRight.setPower(-0.3);
            BackLeft.setPower(-0.3);
            BackRight.setPower(0.3);
        } else {
            FrontLeft.setPower(0.0);
            FrontRight.setPower(0.0);
            BackLeft.setPower(0.0);
            BackRight.setPower(0.0 );
        }

        aButtonPreviousStateG2 = gamepad2.a;
        aButtonPreviousState = gamepad1.a;
        bButtonPreviousStateG2 = gamepad2.b;
        xButtonPreviousStateG2 = gamepad2.x;
        yButtonPreviousStateG2 = gamepad2.y;
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

        FrontLeft.setPower(thetaSpeeds[0] * 0.8);
        FrontRight.setPower(thetaSpeeds[1] * 0.8);
        BackLeft.setPower(thetaSpeeds[2] * 0.8);
        BackRight.setPower(thetaSpeeds[3] * 0.8);
    }
}