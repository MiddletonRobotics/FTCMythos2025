package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeIn extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    boolean elevatorWasMoved;

    public IntakeIn(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.intakeToPosition(IntakeSubsystem.ExtensionState.STORED, IntakeSubsystem.ArmState.STORED, IntakeSubsystem.WristState.NORMAL, IntakeSubsystem.ClawState.CLOSE_CLAW);
        intakeSubsystem.intakeTimer.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.intakeTimer.getElapsedTimeSeconds() > 0.01;
    }
}
