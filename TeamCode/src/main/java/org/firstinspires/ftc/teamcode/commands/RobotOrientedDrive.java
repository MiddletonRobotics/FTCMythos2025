package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class RobotOrientedDrive extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private DoubleSupplier forward, strafe, rotation;
    private boolean squareInputs = false;

    public RobotOrientedDrive(DrivetrainSubsystem drivetrain, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier rotation) {
        this.drivetrain = drivetrain;
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        addRequirements(drivetrain);
    }

    public RobotOrientedDrive squareInputs(boolean squareInputs) {
        this.squareInputs = squareInputs;
        return this;
    }

    @Override
    public void execute() {
        drivetrain.driveRobotCentric(strafe.getAsDouble() * 0.9, forward.getAsDouble() * 0.9, rotation.getAsDouble() * 0.6);
    }
}