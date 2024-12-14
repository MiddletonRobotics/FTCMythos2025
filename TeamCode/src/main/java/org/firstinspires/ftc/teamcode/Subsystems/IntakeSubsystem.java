package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSubsystem {
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
    ArmState armState;
    GrabberState grabberState;

    public IntakeSubsystem(HardwareMap aHardwareMap) {
        linkageServo = aHardwareMap.get(Servo.class, "linkageServo");
        leftArmServo = aHardwareMap.get(Servo.class, "leftPivotServo");
        rightArmServo = aHardwareMap.get(Servo.class, "rightPivotServo");
        wristServo = aHardwareMap.get(Servo.class, "intakeArm");
        grabberServo = aHardwareMap.get(Servo.class, "intakeClaw");

        linkageServo.setDirection(Servo.Direction.FORWARD);
        leftArmServo.setDirection(Servo.Direction.FORWARD);
        rightArmServo.setDirection(Servo.Direction.FORWARD);
        wristServo.setDirection(Servo.Direction.FORWARD);
        grabberServo.setDirection(Servo.Direction.FORWARD);
    }

    public void initialize() {
        extensionState = ExtensionState.STORED;
        armState = ArmState.STORED;
        grabberState = GrabberState.CLOSED;
    }
}
