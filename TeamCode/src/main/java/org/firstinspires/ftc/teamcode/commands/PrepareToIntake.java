package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class PrepareToIntake extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;

    public PrepareToIntake(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.intakeToPosition(
                IntakeSubsystem.ExtensionState.EXTENDED,
                intakeSubsystem.getArmState(),
                IntakeSubsystem.WristState.NORMAL,
                IntakeSubsystem.ClawState.CLOSE_CLAW
        );

        intakeSubsystem.intakeTimer.resetTimer();
    }

    @Override
    public void execute() {
        if(intakeSubsystem.intakeTimer.getElapsedTimeSeconds() > 0.25) {
            intakeSubsystem.intakeToPosition(
                    intakeSubsystem.getExtensionState(),
                    IntakeSubsystem.ArmState.READY,
                    intakeSubsystem.getWristState(),
                    IntakeSubsystem.ClawState.OPEN_CLAW
            );
        }
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.intakeTimer.getElapsedTimeSeconds() > 0.75;
    }
}
