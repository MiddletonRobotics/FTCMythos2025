package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Commands;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.utilities.constants.Constants;

public class LEDSubsystem extends SubsystemBase {
    private final Servo LED;
    private final Telemetry telemetry;

    public Timer ledSubsystemTimer = new Timer();

    private double targetPosition;

    public enum ColorState {
        OFF(Constants.LEDSubsystemOffValue),
        RED(Constants.LEDSubsystemRedValue),
        ORANGE(Constants.LEDSubsystemOrangeValue),
        YELLOW(Constants.LEDSubsystemYellowValue),
        SAGE(Constants.LEDSubsystemSageValue),
        GREEN(Constants.LEDSubsystemGreenValue),
        AZURE(Constants.LEDSubsystemAzureValue),
        BLUE(Constants.LEDSubsystemBlueValue),
        INDIGO(Constants.LEDSubsystemIndigoValue),
        VIOLET(Constants.LEDSubsystemVioletValue),
        WHITE(Constants.LEDSubsystemWhiteValue);

        private final double position;

        private ColorState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }
    }

    private ColorState colorState;

    public LEDSubsystem(HardwareMap aHardwareMap, Telemetry telemetry) {
        LED = aHardwareMap.get(Servo.class, "led");
        colorState = ColorState.OFF;
        targetPosition = colorState.getPosition();

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {

    }

    public void ledToColor(ColorState colorState) {
        setColorState(colorState);
        targetPosition = this.colorState.getPosition();
        LED.setPosition(targetPosition);
    }

    private void setColorState(ColorState colorState) {
        this.colorState = colorState;
    }

    private ColorState getColorState() {
        return colorState;
    }

    public Command flashColor(LEDSubsystem ledSubsystem, ColorState colorState) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> ledSubsystem.ledToColor(colorState)),
                Commands.sleep(75),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.OFF)),
                Commands.sleep(75),
                new InstantCommand(() -> ledSubsystem.ledToColor(colorState)),
                Commands.sleep(75),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.OFF)),
                Commands.sleep(75),
                new InstantCommand(() -> ledSubsystem.ledToColor(colorState)),
                Commands.sleep(75),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.OFF)),
                Commands.sleep(75),
                new InstantCommand(() -> ledSubsystem.ledToColor(colorState)),
                Commands.sleep(75),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.OFF)),
                Commands.sleep(75),
                new InstantCommand(() -> ledSubsystem.ledToColor(colorState)),
                Commands.sleep(75),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.OFF)),
                Commands.sleep(75)

        );
    }
}