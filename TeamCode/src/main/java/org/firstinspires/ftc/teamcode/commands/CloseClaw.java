package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class CloseClaw extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    public CloseClaw(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.manipulatorToPosition(
                elevatorSubsystem.getArmState(),
                elevatorSubsystem.getWristState()   ,
                ElevatorSubsystem.ClawState.CLOSE_CLAW
        );

        elevatorSubsystem.elevatorTimer.resetTimer();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.elevatorTimer.getElapsedTime() > 0.25;
    }
}
