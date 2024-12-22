package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utilities.constants.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final Servo linkageServo, leftArmServo, rightArmServo, wristServo, grabberServo;

    public enum ExtensionState {
        STORED,
        TRANSFER,
        EXTENDED
    }

    public enum ArmState {
        STORED,
        TRANSFER,
        READY,
        INTAKING
    }

    public enum GrabberState {
        CLOSED,
        OPEN,
    }

    ExtensionState extensionState;
    GrabberState grabberState;
    ArmState armState;

    public IntakeSubsystem(HardwareMap aHardwareMap) {
        linkageServo = aHardwareMap.get(Servo.class, "linkageServo");
        leftArmServo = aHardwareMap.get(Servo.class, "leftPivotServo");
        rightArmServo = aHardwareMap.get(Servo.class, "rightPivotServo");
        wristServo = aHardwareMap.get(Servo.class, "intakeArm");
        grabberServo = aHardwareMap.get(Servo.class, "intakeClaw");

        linkageServo.setDirection(Servo.Direction.FORWARD);
        leftArmServo.setDirection(Servo.Direction.REVERSE);
        rightArmServo.setDirection(Servo.Direction.FORWARD);
        wristServo.setDirection(Servo.Direction.FORWARD);
        grabberServo.setDirection(Servo.Direction.FORWARD);
    }

    public class LockedWrist implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                wristServo.setPosition(Constants.intakeArmLockPosition);
                initialized = true;
            }

            return false;
        }
    }

    public Action lockWrist() {
        return new LockedWrist();
    }

    public class OpenClaw implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                grabberServo.setPosition(Constants.intakeClawOpenPosition);
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
                grabberServo.setPosition(Constants.intakeClawClosedPosition);
                initialized = true;
            }

            return false;
        }
    }

    public Action closeClaw() {
        return new CloseClaw();
    }

    public class ExtendIntake implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                linkageServo.setPosition(Constants.linkageServoOutPosition);
                initialized = true;
            }

            return false;
        }
    }

    public Action extendIntake() {
        return new ExtendIntake();
    }

    public class RetractIntake implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                linkageServo.setPosition(Constants.linkageServoInPosition);
                initialized = true;
            }

            return false;
        }
    }

    public Action retractIntake() {
        return new RetractIntake();
    }

    public class StartingPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                closeClaw();
                retractIntake();
                lockWrist();

                leftArmServo.setPosition(Constants.leftPivotServoStorePosition);
                rightArmServo.setPosition(Constants.rightPivotServoStorePosition);
                initialized = true;
            }

            return false;
        }
    }

    public Action startingPosition() {
        return new StartingPosition();
    }

    public class TransferPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                closeClaw();
                retractIntake();
                lockWrist();

                leftArmServo.setPosition(Constants.leftPivotServoTransferPosition);
                rightArmServo.setPosition(Constants.rightPivotServoTransferPosition);
                initialized = true;
            }

            return false;
        }
    }

    public Action transferPosition() {
        return new TransferPosition();
    }

    public class IntakePreperation implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                openClaw();
                extendIntake();
                lockWrist();

                leftArmServo.setPosition(Constants.leftPivotServoIntakeUpPosition);
                rightArmServo.setPosition(Constants.rightPivotServoIntakeUpPosition);
                initialized = true;
            }

            return false;
        }
    }

    public Action intakePreperation() {
        return new IntakePreperation();
    }

    public class IntakingPosition implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                openClaw();
                extendIntake();
                lockWrist();

                leftArmServo.setPosition(Constants.leftPivotServoIntakeDownPosition);
                rightArmServo.setPosition(Constants.rightPivotServoIntakeDownPosition);
                initialized = true;
            }

            return false;
        }
    }

    public Action intakingPosition() {
        return new IntakingPosition();
    }
}
