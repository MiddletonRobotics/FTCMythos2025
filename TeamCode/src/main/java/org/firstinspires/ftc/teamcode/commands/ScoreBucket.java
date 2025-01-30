package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class ScoreBucket extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    public ScoreBucket(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.manipulatorToPosition(
                elevatorSubsystem.getArmState(),
                elevatorSubsystem.getWristState(),
                ElevatorSubsystem.ClawState.OPEN_CLAW
        );

        elevatorSubsystem.elevatorTimer.resetTimer();
    }

    @Override
    public void execute() {
        if(elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() >= 0.25 && elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() < 0.32) {
            elevatorSubsystem.manipulatorToPosition(
                    ElevatorSubsystem.ArmState.TRANSFER,
                    ElevatorSubsystem.WristState.TRANSFER,
                    ElevatorSubsystem.ClawState.OPEN_CLAW
            );
        }
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() > 0.47;
    }
}
