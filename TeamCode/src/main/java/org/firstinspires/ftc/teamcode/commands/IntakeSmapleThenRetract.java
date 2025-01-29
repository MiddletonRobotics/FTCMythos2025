package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeSmapleThenRetract extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    public IntakeSmapleThenRetract(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(elevatorSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.intakeToPosition(
                intakeSubsystem.getExtensionState(),
                intakeSubsystem.getArmState(),
                intakeSubsystem.getWristState(),
                IntakeSubsystem.ClawState.CLOSE_CLAW
        );

        intakeSubsystem.intakeTimer.resetTimer();
    }

    @Override
    public void execute() {
        if (intakeSubsystem.intakeTimer.getElapsedTimeSeconds() >= 0.125 && intakeSubsystem.intakeTimer.getElapsedTimeSeconds() < 0.375) {
            intakeSubsystem.intakeToPosition(
                    intakeSubsystem.getExtensionState(),
                    IntakeSubsystem.ArmState.TRANSFER,
                    IntakeSubsystem.WristState.NORMAL,
                    intakeSubsystem.getClawState()
            );
        } else if (intakeSubsystem.intakeTimer.getElapsedTimeSeconds() >= 0.375 && intakeSubsystem.intakeTimer.getElapsedTimeSeconds() < 1) {
            intakeSubsystem.intakeToPosition(
                    IntakeSubsystem.ExtensionState.STORED,
                    intakeSubsystem.getArmState(),
                    intakeSubsystem.getWristState(),
                    intakeSubsystem.getClawState()
            );

            elevatorSubsystem.elevatorToPosition(ElevatorSubsystem.LiftState.CLEARENCE);
        } else if (intakeSubsystem.intakeTimer.getElapsedTimeSeconds() >= 1 && intakeSubsystem.intakeTimer.getElapsedTimeSeconds() < 1.3) {
            elevatorSubsystem.elevatorToPosition(ElevatorSubsystem.LiftState.RETRACTED);
            elevatorSubsystem.manipulatorToPosition(
                    ElevatorSubsystem.ArmState.TRANSFER,
                    ElevatorSubsystem.WristState.TRANSFER,
                    ElevatorSubsystem.ClawState.OPEN_CLAW
            );
        } else if (intakeSubsystem.intakeTimer.getElapsedTimeSeconds() >= 1.3) {
            elevatorSubsystem.manipulatorToPosition(
                    elevatorSubsystem.getArmState(),
                    elevatorSubsystem.getWristState(),
                    ElevatorSubsystem.ClawState.CLOSE_CLAW
            );

            intakeSubsystem.intakeToPosition(
                    intakeSubsystem.getExtensionState(),
                    intakeSubsystem.getArmState(),
                    intakeSubsystem.getWristState(),
                    IntakeSubsystem.ClawState.PARTIALLY_OPEN_CLAW
            );
        }
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.intakeTimer.getElapsedTimeSeconds() > 1.5;
    }
}
