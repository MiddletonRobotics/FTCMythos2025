package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.utilities.constants.Constants;

public class ScoreSpeciman extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    public ScoreSpeciman(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.manipulatorToPosition(
                ElevatorSubsystem.ArmState.SPECIMAN_SCORE,
                ElevatorSubsystem.WristState.SPECIMAN_SCORE,
                ElevatorSubsystem.ClawState.FULLY_CLOSE_CLAW
        );

        elevatorSubsystem.elevatorTimer.resetTimer();
    }


    @Override
    public void execute() {
        if(elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() < 0.4) {
            elevatorSubsystem.elevatorToPosition(ElevatorSubsystem.LiftState.SPECIMAN_SCORE);
        } else if(elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() >= 0.4 && elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() < 0.45) {
            elevatorSubsystem.manipulatorToPosition(
                    ElevatorSubsystem.ArmState.TRANSFER,
                    ElevatorSubsystem.WristState.TRANSFER,
                    ElevatorSubsystem.ClawState.OPEN_CLAW
            );
        }
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() > 0.45;
    }
}
