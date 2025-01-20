package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class EnableDisableViper extends CommandBase {
    private ElevatorSubsystem elevatorSubsystem;
    private boolean disabled;

    public EnableDisableViper(ElevatorSubsystem elevatorSubsystem, boolean disabled) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.disabled = disabled;

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        if(disabled) {
            elevatorSubsystem.viperMotor.setMotorDisable();
        } else {
            elevatorSubsystem.viperMotor.setMotorEnable();
        }
    }
}
