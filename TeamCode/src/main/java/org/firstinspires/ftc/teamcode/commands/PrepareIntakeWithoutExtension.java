package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class PrepareIntakeWithoutExtension extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    boolean elevatorWasMoved;

    public PrepareIntakeWithoutExtension(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(intakeSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        if (elevatorSubsystem.getLiftState() == ElevatorSubsystem.LiftState.RETRACTED) {
            elevatorSubsystem.manipulatorToPosition(
                    ElevatorSubsystem.ArmState.TRANSFER,
                    ElevatorSubsystem.WristState.TRANSFER,
                    ElevatorSubsystem.ClawState.OPEN_CLAW
            );

            elevatorWasMoved = true;
        } else {
            intakeSubsystem.intakeToPosition(
                    IntakeSubsystem.ExtensionState.EXTENDED,
                    intakeSubsystem.getArmState(),
                    IntakeSubsystem.WristState.NORMAL,
                    IntakeSubsystem.ClawState.CLOSE_CLAW
            );

            elevatorWasMoved = false;
        }

        elevatorSubsystem.elevatorTimer.resetTimer();
        intakeSubsystem.intakeTimer.resetTimer();
    }

    @Override
    public void execute() {
        if(elevatorWasMoved) {
            if(intakeSubsystem.intakeTimer.getElapsedTimeSeconds() < 1) {
                elevatorSubsystem.elevatorToPosition(ElevatorSubsystem.LiftState.CLEARENCE);

                intakeSubsystem.intakeToPosition(
                        IntakeSubsystem.ExtensionState.STORED,
                        IntakeSubsystem.ArmState.READY,
                        IntakeSubsystem.WristState.NORMAL,
                        IntakeSubsystem.ClawState.CLOSE_CLAW
                );
            } else if(intakeSubsystem.intakeTimer.getElapsedTimeSeconds() > 1) {
                elevatorSubsystem.elevatorToPosition(ElevatorSubsystem.LiftState.RETRACTED);
            }
        } else {
            if(intakeSubsystem.intakeTimer.getElapsedTimeSeconds() > 0.375) {
                intakeSubsystem.intakeToPosition(
                        intakeSubsystem.getExtensionState(),
                        IntakeSubsystem.ArmState.READY,
                        intakeSubsystem.getWristState(),
                        IntakeSubsystem.ClawState.CLOSE_CLAW
                );
            }
        }
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.intakeTimer.getElapsedTimeSeconds() > 1;
    }
}
