package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

import java.util.function.DoubleSupplier;

public class FieldOrientedDrive extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private DoubleSupplier forward, strafe, rotation;
    private IMU imu;
    private boolean squareInputs = false;

    public FieldOrientedDrive(DrivetrainSubsystem drivetrain, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier rotation, IMU imu) {
        this.drivetrain = drivetrain;
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;
        this.imu = imu;

        addRequirements(drivetrain);
    }

    public FieldOrientedDrive squareInputs(boolean squareInputs) {
        this.squareInputs = squareInputs;
        return this;
    }

    @Override
    public void execute() {
        drivetrain.driveFieldCentric(strafe.getAsDouble(), forward.getAsDouble(), rotation.getAsDouble(), imu);
    }
}
