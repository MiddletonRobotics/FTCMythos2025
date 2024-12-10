package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="RedAutonomous")
public class RedAutonomous extends OpMode {
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    private DcMotor ViperMotor;

    private Servo Claw;
    private Servo Arm;

    public enum Type {
        PARKING,
        PARKINGFAR,
        SCORE,
        SPECCCCIMANAUTO
    }

    private Type AutonSelection;
    private boolean rightButtonPreviousState;
    private boolean leftButtonPreviousState;
    private boolean aButtonPreviousState;
    private boolean bButtonPreviousState;

    private ElapsedTime RunTime = new ElapsedTime();

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

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop(){
        if(gamepad1.right_bumper && !rightButtonPreviousState) {
            AutonSelection = Type.PARKINGFAR;
        } else if (gamepad1.left_bumper && !leftButtonPreviousState) {
            AutonSelection = Type.PARKING;
        } else if (gamepad1.a && !aButtonPreviousState) {
            AutonSelection = Type.SCORE;
        } else if (gamepad1.b && !aButtonPreviousState) {
            AutonSelection = Type.SPECCCCIMANAUTO;
        }


        rightButtonPreviousState = gamepad1.right_bumper;
        leftButtonPreviousState = gamepad1.left_bumper;
        aButtonPreviousState = gamepad1.a;
        bButtonPreviousState = gamepad1.b;

        telemetry.addData("AutonomousType", AutonSelection);
    };

    @Override
    public void start() {
        RunTime.reset();
    }

    @Override
    public void loop() {
        handle_input();
    }

    void handle_input() {
        if(AutonSelection == Type.PARKINGFAR) {
            if (RunTime.seconds() < 3.35) {
                FrontLeft.setPower(0.4);
                FrontRight.setPower(-0.4);
                BackLeft.setPower(-0.4);
                BackRight.setPower(0.4);
            } else if (RunTime.seconds() > 3.35) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);
            }
        } else if (AutonSelection == Type.PARKING) {
            if (RunTime.seconds() < 1.4) {
                FrontLeft.setPower(0.4);
                FrontRight.setPower(-0.4);
                BackLeft.setPower(-0.4);
                BackRight.setPower(0.4);
            } else if (RunTime.seconds() > 1.4) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);
            }
        } else if (AutonSelection == Type.SCORE) {

            Claw.setPosition(Constants.ClawClosedPosition);
            Arm.setPosition(Constants.ArmOuttakingPosition);
            if (RunTime.seconds() < 0.7) {
                FrontLeft.setPower(0.35);
                FrontRight.setPower(0.35);
                BackLeft.setPower(0.35);
                BackRight.setPower(0.35);
            } else if (RunTime.seconds() > 0.7 && RunTime.seconds() < 3) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);
            } else if (RunTime.seconds() > 3 && RunTime.seconds() < 3.87) {
                FrontLeft.setPower(-0.4);
                FrontRight.setPower(0.4);
                BackLeft.setPower(-0.4);
                BackRight.setPower(0.4);
            } else if (RunTime.seconds() > 3.87 && RunTime.seconds() < 5.8) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);
            } else if (RunTime.seconds() > 5.8 && RunTime.seconds() < 7.5) {
                ViperMotor.setTargetPosition(4200);
                ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ViperMotor.setPower(Constants.ViperConstantVelocity);
                Arm.setPosition(Constants.ArmOuttakingPosition);

                FrontLeft.setPower(0.15);
                FrontRight.setPower(0.15);
                BackLeft.setPower(0.15);
                BackRight.setPower(0.15);
            } else if (RunTime.seconds() > 7.5 && RunTime.seconds() < 10) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);
            } else if (RunTime.seconds() > 10 && RunTime.seconds() < 11.2) {
                FrontLeft.setPower(0.15);
                FrontRight.setPower(0.15);
                BackLeft.setPower(0.15);
                BackRight.setPower(0.15);
            } else if (RunTime.seconds() > 11.2 && RunTime.seconds() < 11.34 ) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);
            } else if (RunTime.seconds() > 11.34 && RunTime.seconds() < 12.5 ) {
                Claw.setPosition(Constants.ClawOpenedPosition);
            } else if (RunTime.seconds() > 12.5 && RunTime.seconds() < 13 ) {
                FrontLeft.setPower(-0.2);
                FrontRight.setPower(-0.2);
                BackLeft.setPower(-0.2);
                BackRight.setPower(-0.2);
            } else if (RunTime.seconds() > 13 && RunTime.seconds() < 16 ) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);

                ViperMotor.setTargetPosition(0);
                ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ViperMotor.setPower(Constants.ViperConstantVelocity);
                Arm.setPosition(Constants.ArmOuttakingPosition);
            } else if (RunTime.seconds() > 16 && RunTime.seconds() < 16.5 ) {
                FrontLeft.setPower(0.2);
                FrontRight.setPower(0.2);
                BackLeft.setPower(0.2);
                BackRight.setPower(0.2);
            }
        }else if (AutonSelection == Type.SPECCCCIMANAUTO) {
            if (RunTime.seconds() < 0.8) {
                Arm.setPosition(Constants.ArmOuttakingPosition);
                Claw.setPosition(Constants.ClawClosedPosition);
                FrontLeft.setPower(0.175);
                FrontRight.setPower(0.175);
                BackLeft.setPower(0.175);
                BackRight.setPower(0.175);
            } else if (RunTime.seconds() > 0.8 && RunTime.seconds() < 2) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);

                ViperMotor.setTargetPosition(Constants.ViperUpperSpecimanPosition);
                ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ViperMotor.setPower(Constants.ViperConstantVelocity);
                Arm.setPosition(Constants.ArmOuttakingPosition);
            } else if (RunTime.seconds() > 2 && RunTime.seconds() < 4.76) {
                FrontLeft.setPower(0.15);
                FrontRight.setPower(0.15);
                BackLeft.setPower(0.15);
                BackRight.setPower(0.15);
            } else if (RunTime.seconds() > 4.76 && RunTime.seconds() < 5.8) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);

                ViperMotor.setTargetPosition(Constants.ViperUpperSpecimanScoringPosition - 200);
                ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ViperMotor.setPower(Constants.ViperConstantVelocity);
                Arm.setPosition(Constants.ArmDeliverPosition);
            } else if (RunTime.seconds() > 5.8 && RunTime.seconds() < 7) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);

                Claw.setPosition(Constants.ClawOpenedPosition);
            } else if (RunTime.seconds() > 7 && RunTime.seconds() < 7.4) {
                FrontLeft.setPower(-0.3);
                FrontRight.setPower(-0.3);
                BackLeft.setPower(-0.3);
                BackRight.setPower(-0.3);
                Claw.setPosition(Constants.ClawOpenedPosition);
            } else if (RunTime.seconds() > 7.4 && RunTime.seconds() < 8) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);

                ViperMotor.setTargetPosition(Constants.ViperRetractedPosition);
                ViperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                ViperMotor.setPower(Constants.ViperConstantVelocity);
                Arm.setPosition(Constants.ArmOuttakingPosition);
                Claw.setPosition(Constants.ClawOpenedPosition);
            } else if (RunTime.seconds() > 8 && RunTime.seconds() < 9.78) {
                FrontLeft.setPower(0.2);
                FrontRight.setPower(-0.25);
                BackLeft.setPower(0.2);
                BackRight.setPower(-0.25);
            }  else if (RunTime.seconds() > 9.78 && RunTime.seconds() < 11) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);
            } else if (RunTime.seconds() > 11 && RunTime.seconds() < 14) {
                FrontLeft.setPower(0.2);
                FrontRight.setPower(0.2);
                BackLeft.setPower(0.2);
                BackRight.setPower(0.2);
                Claw.setPosition(Constants.ClawOpenedPosition);
            }  else if (RunTime.seconds() > 14 && RunTime.seconds() < 15) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);
                Claw.setPosition(Constants.ClawOpenedPosition);
            } else if (RunTime.seconds() > 15 && RunTime.seconds() < 16.83) {
                FrontLeft.setPower(0.2);
                FrontRight.setPower(-0.2);
                BackLeft.setPower(0.2);
                BackRight.setPower(-0.2);
                Claw.setPosition(Constants.ClawOpenedPosition);
            }  else if (RunTime.seconds() > 16.83 && RunTime.seconds() < 17.25) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);
                Claw.setPosition(Constants.ClawOpenedPosition);
            } else if (RunTime.seconds() > 17.25 && RunTime.seconds() < 18.38) {
                FrontLeft.setPower(0.2);
                FrontRight.setPower(0.2);
                BackLeft.setPower(0.2);
                BackRight.setPower(0.2);
            }  else if (RunTime.seconds() > 18.28 && RunTime.seconds() < 18.75) {
                FrontLeft.setPower(0.0);
                FrontRight.setPower(0.0);
                BackLeft.setPower(0.0);
                BackRight.setPower(0.0);
            } else if (RunTime.seconds() > 18.35 && RunTime.seconds() < 18.75) {
                Claw.setPosition(Constants.ClawClosedPosition);
            }
        }
    }
}
