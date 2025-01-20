package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class PrepareSpecimanAuto extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    public PrepareSpecimanAuto(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.manipulatorToPosition(
                ElevatorSubsystem.ArmState.SPECIMAN_READY,
                ElevatorSubsystem.WristState.SPECIMAN_READY,
                ElevatorSubsystem.ClawState.FULLY_CLOSE_CLAW
        );

        elevatorSubsystem.elevatorTimer.resetTimer();
    }

    @Override
    public void execute() {
        elevatorSubsystem.elevatorToPosition(ElevatorSubsystem.LiftState.SPECIMAN_READY);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.elevatorTimer.getElapsedTimeSeconds() > 1.5;
    }
}
