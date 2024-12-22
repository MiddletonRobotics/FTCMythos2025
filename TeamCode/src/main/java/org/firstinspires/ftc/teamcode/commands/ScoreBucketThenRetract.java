package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class ScoreBucketThenRetract extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    public ScoreBucketThenRetract(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setClawState(ElevatorSubsystem.ClawState.OPEN_CLAW);
        elevatorSubsystem.elevatorTimer.resetTimer();
    }

    @Override
    public void execute() {
        if(elevatorSubsystem.elevatorTimer.getElapsedTime() >= 0.25) {
            elevatorSubsystem.setArmState(ElevatorSubsystem.ArmState.TRANSFER);
        } else if (elevatorSubsystem.elevatorTimer.getElapsedTime() >= 0.375) {
            elevatorSubsystem.setLiftState(ElevatorSubsystem.LiftState.RETRACTED);
        }
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.viperAtPosition();
    }
}
