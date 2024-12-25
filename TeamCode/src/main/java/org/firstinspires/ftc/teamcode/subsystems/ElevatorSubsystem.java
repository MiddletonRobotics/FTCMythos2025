package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
        RETRACTED,
        SPECIMAN_READY,
        SPECIMAN_SCORE,
        LOW_GOAL,
        HIGH_GOAL
    }

    public enum ArmState {
        TRANSFER,
        HIGH_GOAL,
        SPECIMAN_READY,
        SPECIMAN_SCORE,
        INTAKING
    }

    public enum ClawState {
        OPEN_CLAW,
        CLOSE_CLAW,
    }

    private LiftState viperState = LiftState.RETRACTED;
    private ArmState manipulatorState = ArmState.SPECIMAN_SCORE;
    private ClawState clawState = ClawState.CLOSE_CLAW;

    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

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

        grabberServo = aHardwareMap.get(Servo.class, "outtakeClaw");
        wristServo = aHardwareMap.get(Servo.class, "outtakeWrist");
        leftArmServo = aHardwareMap.get(Servo.class, "leftouttakeArm");
        rightArmServo = aHardwareMap.get(Servo.class, "rightouttakeArm");

        magneticLimitSwitch = aHardwareMap.get(RevTouchSensor.class, "magneticLimitSwitch");

        grabberServo.setDirection(Servo.Direction.REVERSE);
        wristServo.setDirection(Servo.Direction.FORWARD);
        leftArmServo.setDirection(Servo.Direction.REVERSE);
        rightArmServo.setDirection(Servo.Direction.FORWARD);

        elevatorController = new PIDFController(0.001, 0, 0.0001, 0);
        elevatorController.setTolerance(50);

        targetPosition = 0;
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        //handleElevatorPID();
        viperMotor.setTargetPosition((int) targetPosition);
        viperMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        viperMotor.setPower(0.7);

        TelemetryPacket packet = new TelemetryPacket();

        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }

        runningActions = newActions;
        dash.sendTelemetryPacket(packet);

        switch (viperState) {
            case RETRACTED:
                targetPosition = Constants.ViperRetractedPosition;
            case SPECIMAN_READY:
                targetPosition = Constants.ViperSpecimanReadyPosition;
            case SPECIMAN_SCORE:
                targetPosition = Constants.ViperSpecimanScorePosition;
            case HIGH_GOAL:
                targetPosition = Constants.ViperHighGoalPosition;
            case LOW_GOAL:
                targetPosition = Constants.ViperLowGoalPosition;
            default:
                targetPosition = 0;
        }

        switch (manipulatorState){
            case TRANSFER:
                transfer();
            case HIGH_GOAL:
                prepareToScore();
            case INTAKING:
                intakingPosition();
            case SPECIMAN_READY:
            case SPECIMAN_SCORE:
        }

        switch (clawState){
            case OPEN_CLAW:
                runningActions.add(openClaw());
            case CLOSE_CLAW:
                runningActions.add(closeClaw());
        }

        telemetry.addData("Viper Current Position", getViperPosition());
        telemetry.addData("Viper Target Position", targetPosition);
        telemetry.addData("Viper Power", viperMotor.getPower());
        telemetry.addData("Viper State", viperState);
        telemetry.addData("Arm State", manipulatorState);
        telemetry.addData("Claw State", clawState);
        telemetry.update();
    }

    public void handleElevatorPID() {
        if (targetPosition == 0) {
            viperMotor.setPower(0.0);
            return;
        }

        double output = elevatorController.calculate(viperMotor.getCurrentPosition(), targetPosition);
        setViperPower(output);
    }

    public int getViperPosition() {
        return viperMotor.getCurrentPosition();
    }

    public void setLiftState(LiftState liftState) {
        viperState = liftState;
    }

    public void setArmState(ArmState armState) {
        manipulatorState = armState;
    }

    public void setClawState(ClawState clawState) {
        this.clawState = clawState;
    }

    public void setViperPower(double input) {
        viperMotor.setPower(input);
    }

    public boolean viperAtPosition() {
        return elevatorController.atSetPoint();
    }

    public class ElevatorHighBucketPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                viperState = LiftState.HIGH_GOAL;
                initialized = true;
            }

            double currentPosition = viperMotor.getCurrentPosition();
            telemetryPacket.put("viperPosition", currentPosition);
            if (elevatorController.atSetPoint()) {
                return false;
            } else {
                periodic();
                return true;
            }
        }
    }

    public Action elevatorHighBucketPosition() {
        return new ElevatorHighBucketPosition();
    }

    public class ElevatorLowBucketPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                viperState = LiftState.LOW_GOAL;
                initialized = true;
            }

            double currentPosition = viperMotor.getCurrentPosition();
            telemetryPacket.put("viperPosition", currentPosition);
            if (elevatorController.atSetPoint()) {
                return false;
            } else {
                periodic();
                return true;
            }
        }
    }

    public Action elevatorLowBucketPosition() {
        return new ElevatorLowBucketPosition();
    }

    public class ElevatorSpecimanScorePosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                viperState = LiftState.SPECIMAN_SCORE;
                initialized = true;
            }

            double currentPosition = viperMotor.getCurrentPosition();
            telemetryPacket.put("viperPosition", currentPosition);
            if (elevatorController.atSetPoint()) {
                return false;
            } else {
                periodic();
                return true;
            }
        }
    }

    public Action elevatorSpecimanScorePosition() {
        return new ElevatorSpecimanScorePosition();
    }

    public class ElevatorSpecimanReadyPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                viperState = LiftState.SPECIMAN_READY;
                initialized = true;
            }

            double currentPosition = viperMotor.getCurrentPosition();
            telemetryPacket.put("viperPosition", currentPosition);
            if (elevatorController.atSetPoint()) {
                return false;
            } else {
                periodic();
                return true;
            }
        }
    }

    public Action elevatorSpecimanReadyPosition() {
        return new ElevatorSpecimanReadyPosition();
    }

    public class ElevatorRetractedPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                viperState = LiftState.RETRACTED;
                initialized = true;
            }

            double currentPosition = viperMotor.getCurrentPosition();
            telemetryPacket.put("viperPosition", currentPosition);
            if (elevatorController.atSetPoint()) {
                return false;
            } else {
                periodic();
                return true;
            }
        }
    }

    public Action elevatorRetractedPosition() {
        return new ElevatorRetractedPosition();
    }

    public class ElevatorHoming implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                viperMotor.setPower(-0.2);
                initialized = true;
            }

            boolean isViperHomed = magneticLimitSwitch.isPressed();
            telemetryPacket.put("isViperHomed", isViperHomed);
            if (magneticLimitSwitch.isPressed()) {
                viperMotor.setPower(0.0);
                return false;
            } else {
                return true;
            }
        }
    }

    public Action homeElevator() {
        return new ElevatorHoming();
    }

    /* Outtake Servo Positions  */

    public class OpenClaw implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                grabberServo.setPosition(Constants.outtakeClawOpenPosition);
                initialized = true;
            }

            return false;
        }
    }

    public Action openClaw() {
        return new OpenClaw();
    }

    public class CloseClaw implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                grabberServo.setPosition(Constants.outtakeClawClosedPosition);
                initialized = true;
            }

            return false;
        }
    }

    public Action closeClaw() {
        return new CloseClaw();
    }

    public class SpecimanIntakingPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                closeClaw();
                leftArmServo.setPosition(Constants.leftouttakeArmIntakePosition);
                rightArmServo.setPosition(Constants.rightouttakeArmIntakePosition);
                wristServo.setPosition(Constants.outtakeWristIntakePosition);
                initialized = true;
            }

            return false;
        }
    }

    public Action intakingPosition() {
        return new SpecimanIntakingPosition();
    }

    public class PrepareToScore implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                closeClaw();
                leftArmServo.setPosition(Constants.leftouttakeArmScorePosition);
                rightArmServo.setPosition(Constants.rightouttakeArmScorePosition);
                wristServo.setPosition(Constants.outtakeWristScorePosition);
                initialized = true;
            }

            return false;
        }
    }

    public Action prepareToScore() {
        return new PrepareToScore();
    }

    public class Transfer implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                closeClaw();
                leftArmServo.setPosition(Constants.leftouttakeArmTransferPosition);
                rightArmServo.setPosition(Constants.rightouttakeArmTransferPosition);
                wristServo.setPosition(Constants.outtakeWristTransferPosition);
                initialized = true;
            }

            return false;
        }
    }

    public Action transfer() {
        return new Transfer();
    }
}