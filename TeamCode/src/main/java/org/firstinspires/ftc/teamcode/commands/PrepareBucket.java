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
        elevatorSubsystem.setLiftState(ElevatorSubsystem.LiftState.HIGH_GOAL);
        elevatorSubsystem.setClawState(ElevatorSubsystem.ClawState.CLOSE_CLAW);
        elevatorSubsystem.elevatorTimer.resetTimer();
    }

    @Override
    public void execute() {
        if(elevatorSubsystem.elevatorTimer.getElapsedTime() >= 0.8) {
            elevatorSubsystem.setArmState(ElevatorSubsystem.ArmState.HIGH_GOAL);
        }
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.viperAtPosition();
    }
}
