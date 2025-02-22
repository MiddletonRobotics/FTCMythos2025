package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.utilities.constants.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final Servo linkageServo, linkageServo2, leftArmServo, rightArmServo, wristServo, grabberServo;
    private Telemetry telemetry;

    public Timer intakeTimer = new Timer();

    public enum ExtensionState {
        STORED(Constants.IntakeLinkageServoInPosition),
        TRANSFER(Constants.IntakeLinkageServoTransferPosition),
        EXTENDED(Constants.IntakeLinkageServoOutPosition);

        private final double position;

        private ExtensionState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

    public enum ArmState {
        STORED(Constants.IntakeArmStorePosition),
        TRANSFER(Constants.IntakeArmTransferPosition),
        READY(Constants.IntakeArmReadyPosition),
        INTAKING(Constants.IntakeArmIntakingPosition);

        private final double position;

        private ArmState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

    public enum WristState {
        NORMAL(Constants.IntakeWristRegularPosition),
        ANGLED_30(Constants.IntakeWristAngled30Position),
        ANGLED_60(Constants.IntakeWristAngled60Position),
        ANGLED_90(Constants.IntakeWristAngled90Position);

        private final double position;

        private WristState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

    public enum ClawState {
        OPEN_CLAW(Constants.IntakeClawOpenedPosition),
        PARTIALLY_OPEN_CLAW(Constants.IntakeClawPartiallyOpenedPosition),
        CLOSE_CLAW(Constants.IntakeClawClosedPosition);

        private final double position;

        private ClawState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

    ExtensionState extensionState = ExtensionState.STORED;
    ArmState armState = ArmState.STORED;
    WristState wristState = WristState.NORMAL;
    ClawState clawState = ClawState.OPEN_CLAW;

    public IntakeSubsystem(HardwareMap aHardwareMap, Telemetry telemetry) {
        linkageServo = aHardwareMap.get(Servo.class, "linkageServo");
        linkageServo2 = aHardwareMap.get(Servo.class, "linkageServo2");
        leftArmServo = aHardwareMap.get(Servo.class, "leftPivotServo");
        rightArmServo = aHardwareMap.get(Servo.class, "rightPivotServo");
        wristServo = aHardwareMap.get(Servo.class, "intakeArm");
        grabberServo = aHardwareMap.get(Servo.class, "intakeClaw");

        linkageServo.setDirection(Servo.Direction.FORWARD);
        linkageServo2.setDirection(Servo.Direction.REVERSE);
        leftArmServo.setDirection(Servo.Direction.REVERSE);
        rightArmServo.setDirection(Servo.Direction.FORWARD);
        wristServo.setDirection(Servo.Direction.FORWARD);
        grabberServo.setDirection(Servo.Direction.FORWARD);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        /*
        telemetry.addData("Extension State", extensionState);
        telemetry.addData("Arm State", armState);
        telemetry.addData("Wrist State", wristState);
        telemetry.addData("Claw State", clawState);
        telemetry.update();
         */
    }

    public void intakeToPosition(ExtensionState extensionState, ArmState armState, WristState wristState, ClawState clawState) {
        setExtensionState(extensionState);
        setArmState(armState);
        setWristState(wristState);
        setClawState(clawState);

        linkageServo.setPosition(this.extensionState.getPosition());
        linkageServo2.setPosition(this.extensionState.getPosition());
        leftArmServo.setPosition(this.armState.getPosition());
        rightArmServo.setPosition(this.armState.getPosition());
        wristServo.setPosition(this.wristState.getPosition());
        grabberServo.setPosition(this.clawState.getPosition());
    }

    public ExtensionState getExtensionState() {
        return extensionState;
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

    private void setExtensionState(ExtensionState extensionState) {
        this.extensionState = extensionState;
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
}
