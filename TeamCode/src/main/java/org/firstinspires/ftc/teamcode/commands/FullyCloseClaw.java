package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class FullyCloseClaw extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    public FullyCloseClaw(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.manipulatorToPosition(
                elevatorSubsystem.getArmState(),
                elevatorSubsystem.getWristState()   ,
                ElevatorSubsystem.ClawState.FULLY_CLOSE_CLAW
        );

        elevatorSubsystem.elevatorTimer.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.elevatorTimer.getElapsedTime() > 0.25;
    }
}
