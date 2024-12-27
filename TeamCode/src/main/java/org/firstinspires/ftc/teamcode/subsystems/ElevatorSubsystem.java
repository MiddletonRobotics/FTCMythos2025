package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Commands;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.utilities.PIDFController;
import org.firstinspires.ftc.teamcode.utilities.constants.Constants;

import java.util.ArrayList;
import java.util.List;

public class ElevatorSubsystem extends SubsystemBase {
    private final Servo leftArmServo, rightArmServo, wristServo, grabberServo;
    private RevTouchSensor magneticLimitSwitch;
    private final PIDFController elevatorController;
    private Telemetry telemetry;
    private double targetPosition;
    DcMotorEx viperMotor;

    public Timer elevatorTimer = new Timer();

    public enum LiftState {
        RETRACTED(Constants.ViperRetractedPosition),
        SPECIMAN_READY(Constants.ViperSpecimanReadyPosition),
        SPECIMAN_SCORE(Constants.ViperSpecimanScorePosition),
        LOW_GOAL(Constants.ViperLowGoalPosition),
        HIGH_GOAL(Constants.ViperHighGoalPosition);

        private final double position;

        private LiftState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

    public enum ArmState {
        TRANSFER(Constants.ElevatorArmTransferPosition),
        BUCKET(Constants.ElevatorArmBucketPosition),
        SPECIMAN_READY(Constants.ElevatorArmSpecimanPosition),
        SPECIMAN_SCORE(Constants.ElevatorArmSpecimanPosition),
        INTAKING(Constants.ElevatorArmIntakingPosition);

        private final double position;

        private ArmState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

    public enum WristState {
        TRANSFER(Constants.ElevatorWristTransferPosition),
        BUCKET(Constants.ElevatorWristBucketPosition),
        SPECIMAN_READY(Constants.ElevatorWristSpecimanPosition),
        SPECIMAN_SCORE(Constants.ElevatorWristSpecimanPosition),
        INTAKING(Constants.ElevatorWristIntakingPosition);

        private final double position;

        private WristState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

    public enum ClawState {
        OPEN_CLAW(Constants.ElevatorClawOpenPosition),
        CLOSE_CLAW(Constants.ElevatorClawClosedPosition);

        private final double position;

        private ClawState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

    private LiftState viperState = LiftState.RETRACTED;
    private ArmState armState = ArmState.TRANSFER;
    private WristState wristState = WristState.TRANSFER;
    private ClawState clawState = ClawState.CLOSE_CLAW;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    /**
     * The constructor for the elevator subsystem.
     *
     * @param aHardwareMap Passing through HardwareMap from OpMode
     */

    public ElevatorSubsystem(HardwareMap aHardwareMap, Telemetry telemetry) {
        viperMotor = aHardwareMap.get(DcMotorEx.class, Constants.ViperMotorID);
        viperMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        viperMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viperMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        grabberServo = aHardwareMap.get(Servo.class, "outtakeClaw");
        wristServo = aHardwareMap.get(Servo.class, "outtakeWrist");
        leftArmServo = aHardwareMap.get(Servo.class, "leftouttakeArm");
        rightArmServo = aHardwareMap.get(Servo.class, "rightouttakeArm");

        magneticLimitSwitch = aHardwareMap.get(RevTouchSensor.class, "magneticLimitSwitch");

        grabberServo.setDirection(Servo.Direction.REVERSE);
        wristServo.setDirection(Servo.Direction.FORWARD);
        leftArmServo.setDirection(Servo.Direction.REVERSE);
        rightArmServo.setDirection(Servo.Direction.FORWARD);

        elevatorController = new PIDFController(0.016, 0, 0.00008, 0);
        elevatorController.setTolerance(20);

        targetPosition = 0;
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData("Viper Current Position", getViperPosition());
        telemetry.addData("Viper Target Position", targetPosition);
        telemetry.addData("Viper Power", viperMotor.getPower());
        telemetry.addData("Viper State", viperState);
        telemetry.addData("Arm State", armState);
        telemetry.addData("Wrist State", wristState);
        telemetry.addData("Claw State", clawState);
        telemetry.update();
    }

    public void elevatorToPosition(LiftState liftState) {
        setLiftState(liftState);
        targetPosition = viperState.getPosition();
        viperMotor.setPower(elevatorController.calculate(getViperPosition(), targetPosition));
    }

    public void manipulatorToPosition(ArmState armState, WristState wristState, ClawState clawState) {
        setArmState(armState);
        setWristState(wristState);
        setClawState(clawState);

        leftArmServo.setPosition(this.armState.getPosition());
        rightArmServo.setPosition(this.armState.getPosition());
        wristServo.setPosition(this.wristState.getPosition());
        grabberServo.setPosition(this.clawState.getPosition());
    }

    private void setLiftState(LiftState liftState) {
        viperState = liftState;
    }

    private void setArmState(ArmState armState) {
        this.armState = armState;
    }

    private void setWristState(WristState wristState) {
        this.wristState = wristState;
    }

    private void setClawState(ClawState clawState) {
        this.clawState = clawState;
    }

    public LiftState getLiftState() {
        return viperState;
    }

    public ArmState getArmState() {
        return armState;
    }

    public WristState getWristState() {
        return wristState;
    }

    public ClawState getClawState() {
        return clawState;
    }

    public int getViperPosition() {
        return viperMotor.getCurrentPosition();
    }

    public double getTargetPosition() {
        return elevatorController.getSetPoint();
    }


    public boolean viperAtTarget() {
        return elevatorController.atSetPoint();
    }
}