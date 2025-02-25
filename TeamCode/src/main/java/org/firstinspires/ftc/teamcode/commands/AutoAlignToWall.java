package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class AutoAlignToWall extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;

    public static double target = 0.8;
    private double lastErrorDistance, errorDistance, distance = 0;
    private double correctionY;

    private static final double kPtx = 0.155;
    private static final double kDtx = 0.03;

    public AutoAlignToWall(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.follower.startTeleopDrive();
    }

    @Override
    public void execute() {
        drivetrainSubsystem.follower.update();

        distance = drivetrainSubsystem.getRearUltrasonicDistance();
        errorDistance = distance - target;

        double derivativeDistance = errorDistance - lastErrorDistance;
        lastErrorDistance = errorDistance;

        correctionY = (kPtx * errorDistance) + (kDtx * derivativeDistance);

        drivetrainSubsystem.driveRobotPedroFollower(0, -correctionY, 0);
    }
}
