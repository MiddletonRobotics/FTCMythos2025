package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.Utilities.constants.Constants;

@TeleOp
public class PositioningTeleop extends OpMode {
    private DrivetrainSubsystem drivetrain;
    private GamepadEx driverController;
    private RevIMU imu;
    private static boolean isRobotRelative = true;
    private Servo intakeClaw, intakeArm, leftPivotServo, rightPivotServo, linkageServo, outtakeClaw, rightouttakeArm, leftouttakeArm, outtakeWrist ;

    @Override
    public void init() {
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        outtakeClaw.setDirection((Servo.Direction.REVERSE));

        rightouttakeArm = hardwareMap.get(Servo.class, "rightouttakeArm");
        rightouttakeArm.setDirection((Servo.Direction.FORWARD));

        leftouttakeArm = hardwareMap.get(Servo.class, "leftouttakeArm");
        leftouttakeArm.setDirection((Servo.Direction.REVERSE));

        outtakeWrist = hardwareMap.get(Servo.class, "outtakeWrist");
        outtakeWrist.setDirection((Servo.Direction.REVERSE));

        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");
        intakeClaw.setDirection(Servo.Direction.FORWARD);

        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        intakeArm.setDirection((Servo.Direction.FORWARD));

        leftPivotServo = hardwareMap.get(Servo.class, "leftPivotServo");
        leftPivotServo.setDirection((Servo.Direction.FORWARD));

        rightPivotServo = hardwareMap.get(Servo.class, "rightPivotServo");
        rightPivotServo.setDirection((Servo.Direction.REVERSE));

        linkageServo = hardwareMap.get(Servo.class, "linkageServo");
        linkageServo.setDirection((Servo.Direction.FORWARD));
    }

    @Override
    public void loop() {
        //leftouttakeArm.setPosition(0);
        //rightouttakeArm.setPosition(0);
        outtakeWrist.setPosition(0.9);

        leftPivotServo.setPosition(0.0);
        rightPivotServo.setPosition(0.0);

        telemetry.addData( "outtakeClaw current pos", outtakeClaw.getPosition());
        telemetry.addData( "rightouttakeArm current pos", rightouttakeArm.getPosition());
        telemetry.addData( "leftouttakeArm current pos", leftouttakeArm.getPosition());
        telemetry.addData( "outtakeWrist current pos", outtakeWrist.getPosition());
        telemetry.addData( "intakeClaw current pos", intakeClaw.getPosition());
        telemetry.addData( "intakeArm current pos", intakeArm.getPosition());
        telemetry.addData( "leftPivotServo current pos", leftPivotServo.getPosition());
        telemetry.addData( "rightPivotServo current pos", rightPivotServo.getPosition());
        telemetry.addData( "linkageServo current pos", linkageServo.getPosition());
    }
}
