package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "Telop")
public class Teleop extends OpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;

    private Servo Claw;
    private Servo rollerClaw;

    private boolean aButtonPreviousState;
    private boolean bButtonPreviousState;
    private boolean xButtonPreviousState;
    private boolean SlowMode;

    @Override
    public void init() {
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        Claw = hardwareMap.get(Servo.class, "Claw");
        Claw.setDirection(Servo.Direction.REVERSE);

        rollerClaw = hardwareMap.get(Servo.class, "rollerClaw");
        rollerClaw.setDirection(Servo.Direction.REVERSE);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        aButtonPreviousState = false;
        bButtonPreviousState = false;
        xButtonPreviousState = false;
        SlowMode = false;

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop(){};

    @Override
    public void start(){
        resetRuntime();
        Claw.setPosition(0.0);
        rollerClaw.setPosition(1.0);
    };

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotation = -gamepad1.right_stick_x;

        telemetry.addData("Controller Left Y", -gamepad1.left_stick_y);
        telemetry.addData("Controller Left X", gamepad1.left_stick_x);
        telemetry.addData("Controller Right X", gamepad1.right_stick_x);
        telemetry.addData("Claw Position", Claw.getPosition());

        if(gamepad1.a && !aButtonPreviousState) {
            Claw.setPosition(0.03);
        } else if (gamepad1.b && bButtonPreviousState) {
            Claw.setPosition(0.55);
        }

        if(gamepad1.x && !xButtonPreviousState) {
            SlowMode = !SlowMode;
        }

        aButtonPreviousState = gamepad1.a;
        bButtonPreviousState = gamepad1.b;
        xButtonPreviousState = gamepad1.x;

        double[] thetaSpeeds = {
                (drive + strafe + rotation),
                (drive - strafe - rotation),
                (drive - strafe + rotation),
                (drive + strafe - rotation),
        };

        if(SlowMode) {
            FrontLeft.setPower(thetaSpeeds[0] * 0.25);
            FrontRight.setPower(thetaSpeeds[1] * 0.25);
            BackLeft.setPower(thetaSpeeds[2] * 0.25);
            BackRight.setPower(thetaSpeeds[3] * 0.25);
        } else {
            FrontLeft.setPower(thetaSpeeds[0]);
            FrontRight.setPower(thetaSpeeds[1]);
            BackLeft.setPower(thetaSpeeds[2]);
            BackRight.setPower(thetaSpeeds[3]);
        }
    }
}
