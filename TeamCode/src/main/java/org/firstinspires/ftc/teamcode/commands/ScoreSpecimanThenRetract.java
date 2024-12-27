package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.utilities.constants.Constants;

public class ScoreSpecimanThenRetract extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    public ScoreSpecimanThenRetract(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.manipulatorToPosition(
                ElevatorSubsystem.ArmState.SPECIMAN_SCORE,
                ElevatorSubsystem.WristState.SPECIMAN_SCORE,
                elevatorSubsystem.getClawState()
        );

        elevatorSubsystem.elevatorTimer.resetTimer();
    }

    @Override
    public void execute() {
        if(elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() < 0.75) {
            elevatorSubsystem.elevatorToPosition(ElevatorSubsystem.LiftState.SPECIMAN_SCORE);
        } else if(elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() >= 0.75 && elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() < 1) {
            elevatorSubsystem.manipulatorToPosition(
                    elevatorSubsystem.getArmState(),
                    elevatorSubsystem.getWristState(),
                    ElevatorSubsystem.ClawState.OPEN_CLAW
            );
        } else if (elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() >= 1) {
            elevatorSubsystem.elevatorToPosition(ElevatorSubsystem.LiftState.RETRACTED);
        }
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() > 3;
    }
}
