package org.firstinspires.ftc.teamcode.Subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.PIDFController;
import org.firstinspires.ftc.teamcode.Utilities.constants.Constants;

public class ElevatorSubsystem extends SubsystemBase {
    private final Servo leftArmServo, rightArmServo, wristServo, grabberServo;
    private final PIDFController elevatorController;
    private Telemetry telemetry;
    private double targetPosition;
    DcMotorEx viperMotor;

    public enum LiftState {
        RETRACTED,
        SPECIMAN_READY,
        SPECIMAN_SCORE,
        LOW_GOAL,
        HIGH_GOAL
    }

    LiftState viperState = LiftState.RETRACTED;

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

        grabberServo.setDirection(Servo.Direction.REVERSE);
        wristServo.setDirection(Servo.Direction.FORWARD);
        leftArmServo.setDirection(Servo.Direction.REVERSE);
        rightArmServo.setDirection(Servo.Direction.FORWARD);

        elevatorController = new PIDFController(0.001, 0, 0.0001, 0);
        elevatorController.setTolerance(50);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        viperMotor.setPower(elevatorController.calculate(viperMotor.getCurrentPosition(), targetPosition));

        switch (viperState){
            case RETRACTED:
                targetPosition = Constants.ViperRetractedPosition;
            case SPECIMAN_READY:
                targetPosition = Constants.ViperSpecimanReadyPosition;
            case SPECIMAN_SCORE:
                targetPosition = Constants.ViperSpecimanScorePosition;
            case LOW_GOAL:
                targetPosition = Constants.ViperLowGoalPosition;
            case HIGH_GOAL:
                targetPosition = Constants.ViperHighGoalPosition;
        }

        telemetry.addData("Viper Current Position", viperMotor.getCurrentPosition());
        telemetry.addData("Viper Target Position", targetPosition);
        telemetry.addData("Viper State", viperState);
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
}
