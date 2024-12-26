package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

// TODO: Fix the isFinished() function

public class IntakeFromWall extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    public IntakeFromWall(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setArmState(ElevatorSubsystem.ArmState.INTAKING);
        elevatorSubsystem.setClawState(ElevatorSubsystem.ClawState.OPEN_CLAW);
        elevatorSubsystem.elevatorTimer.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.viperAtPosition();
    }
}
