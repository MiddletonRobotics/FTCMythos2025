package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;

public class LEDCommands {
    public static Command flashGreen(LEDSubsystem ledSubsystem) {
        return new SequentialCommandGroup(
                new RunCommand(() -> ledSubsystem.ledSubsystemTimer.resetTimer()),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.GREEN)),
                Commands.sleep(500),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.OFF)),
                Commands.sleep(500),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.GREEN)),
                Commands.sleep(500),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.OFF)),
                Commands.sleep(500),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.GREEN)),
                Commands.sleep(500),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.OFF)),
                Commands.sleep(500)
        );
    }

    public static Command flashRed(LEDSubsystem ledSubsystem) {
        return new SequentialCommandGroup(
                new RunCommand(() -> ledSubsystem.ledSubsystemTimer.resetTimer()),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.RED)),
                Commands.sleep(500),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.OFF)),
                Commands.sleep(500),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.RED)),
                Commands.sleep(500),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.OFF)),
                Commands.sleep(500),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.RED)),
                Commands.sleep(500),
                new InstantCommand(() -> ledSubsystem.ledToColor(LEDSubsystem.ColorState.OFF)),
                Commands.sleep(500)
        );
    }
}
