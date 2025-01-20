package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class RetractThenIntake extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    public RetractThenIntake(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.elevatorToPosition(ElevatorSubsystem.LiftState.RETRACTED);
        elevatorSubsystem.manipulatorToPosition(
                ElevatorSubsystem.ArmState.INTAKING,
                ElevatorSubsystem.WristState.INTAKING,
                ElevatorSubsystem.ClawState.OPEN_CLAW
        );
        elevatorSubsystem.elevatorTimer.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getViperPosition() < 1.25;
    }
}