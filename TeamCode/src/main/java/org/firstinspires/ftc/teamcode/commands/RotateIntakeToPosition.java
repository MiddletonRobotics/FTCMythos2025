package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class RotateIntakeToPosition extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public RotateIntakeToPosition(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.intakeToPosition(intakeSubsystem.getExtensionState(), intakeSubsystem.getArmState(), IntakeSubsystem.WristState.ANGLED_30, intakeSubsystem.getClawState());
        intakeSubsystem.intakeTimer.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.intakeTimer.getElapsedTimeSeconds() > 0.125;
    }
}
