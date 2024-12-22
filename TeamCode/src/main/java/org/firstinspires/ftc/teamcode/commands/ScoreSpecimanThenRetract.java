package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.utilities.constants.Constants;

public class ScoreSpecimanThenRetract extends CommandBase {
    private final ElevatorSubsystem elevatorSubsystem;

    public ScoreSpecimanThenRetract(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setLiftState(ElevatorSubsystem.LiftState.SPECIMAN_SCORE);
        elevatorSubsystem.setArmState(ElevatorSubsystem.ArmState.SPECIMAN_SCORE);
        elevatorSubsystem.elevatorTimer.resetTimer();
    }

    @Override
    public void execute() {
        if(elevatorSubsystem.elevatorTimer.getElapsedTime() >= 0.375) {
            elevatorSubsystem.setClawState(ElevatorSubsystem.ClawState.OPEN_CLAW);
            elevatorSubsystem.setLiftState(ElevatorSubsystem.LiftState.RETRACTED);
        }
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getViperPosition() < Constants.ViperSpecimanReadyPosition - 400;
    }
}
