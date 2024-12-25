package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class RetractElevator extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    public RetractElevator(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setLiftState(ElevatorSubsystem.LiftState.RETRACTED);
        elevatorSubsystem.elevatorTimer.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.viperAtPosition();
    }
}
