package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class AutoAlignToWall extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;
    private Follower follower;

    public static double target = 0.8;
    private double lastErrorDistance, errorDistance, distance = 0;
    private double correctionY;

    private static final double kPtx = 0.255;
    private static final double kDtx = 0.028;

    private Timer timer;

    public  AutoAlignToWall(DrivetrainSubsystem drivetrainSubsystem, Follower follower) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.follower = follower;
        timer = new Timer();

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        follower.startTeleopDrive();
        timer.resetTimer();
    }

    @Override
    public void execute() {
        distance = drivetrainSubsystem.getRearUltrasonicDistance();
        errorDistance = distance - target;

        double derivativeDistance = errorDistance - lastErrorDistance;
        lastErrorDistance = errorDistance;

        correctionY = (kPtx * errorDistance) + (kDtx * derivativeDistance);

        drivetrainSubsystem.driveRobotCentric(0, -correctionY, 0);
    }

    @Override
    public boolean isFinished() {
        return timer.getElapsedTimeSeconds() > 0.4;
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            follower.breakFollowing();
            drivetrainSubsystem.pickupPosition = new Pose(drivetrainSubsystem.follower.getPose().getX(), drivetrainSubsystem.follower.getPose().getY(), 0);
        }
    }
}
