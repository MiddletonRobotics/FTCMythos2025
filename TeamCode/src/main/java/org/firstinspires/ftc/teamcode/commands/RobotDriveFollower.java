package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class RobotDriveFollower extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private DoubleSupplier forward, strafe, rotation;
    private boolean squareInputs = false;

    public RobotDriveFollower(DrivetrainSubsystem drivetrain, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier rotation) {
        this.drivetrain = drivetrain;
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.startPedroPathing();
    }


    @Override
    public void execute() {
        drivetrain.driveRobotPedroFollower(strafe.getAsDouble() * 0.9, forward.getAsDouble() * 0.9, rotation.getAsDouble() * 0.6);
    }
}