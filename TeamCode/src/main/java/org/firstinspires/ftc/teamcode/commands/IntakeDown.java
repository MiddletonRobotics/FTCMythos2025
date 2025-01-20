package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

// TODO: Fix the isFinished() function

public class IntakeDown extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public IntakeDown(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.intakeToPosition(
                intakeSubsystem.getExtensionState(),
                IntakeSubsystem.ArmState.INTAKING,
                intakeSubsystem.getWristState(),
                IntakeSubsystem.ClawState.OPEN_CLAW
        );

        intakeSubsystem.intakeTimer.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.intakeTimer.getElapsedTime() > 0.5;
    }
}
