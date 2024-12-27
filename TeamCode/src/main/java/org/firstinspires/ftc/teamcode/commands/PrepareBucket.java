package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class PrepareBucket extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    public PrepareBucket(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.elevatorTimer.resetTimer();
    }

    @Override
    public void execute() {
        elevatorSubsystem.elevatorToPosition(ElevatorSubsystem.LiftState.HIGH_GOAL);

        if(elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() >= 0.8) {
            elevatorSubsystem.manipulatorToPosition(
                    ElevatorSubsystem.ArmState.BUCKET,
                    ElevatorSubsystem.WristState.BUCKET,
                    ElevatorSubsystem.ClawState.CLOSE_CLAW
            );
        }
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() > 2.1;
    }
}
