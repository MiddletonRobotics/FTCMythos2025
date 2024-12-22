package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class PrepareSpeciman extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    public PrepareSpeciman(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setLiftState(ElevatorSubsystem.LiftState.SPECIMAN_READY);
        elevatorSubsystem.setArmState(ElevatorSubsystem.ArmState.SPECIMAN_READY);
        elevatorSubsystem.setClawState(ElevatorSubsystem.ClawState.CLOSE_CLAW);
        elevatorSubsystem.elevatorTimer.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.viperAtPosition();
    }
}
