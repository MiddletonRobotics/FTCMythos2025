package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

import java.util.function.DoubleSupplier;

public class RobotDriveFollower extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private ElevatorSubsystem elevator;
    private DoubleSupplier forward, strafe, rotation;
    private boolean squareInputs = false;
    private PathChain chain;

    public RobotDriveFollower(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, DoubleSupplier strafe, DoubleSupplier forward, DoubleSupplier rotation) {
        this.drivetrain = drivetrain;
        this.elevator = elevator;
        this.forward = forward;
        this.strafe = strafe;
        this.rotation = rotation;

        chain = drivetrain.getAutoScoringPath();

        addRequirements(drivetrain, elevator);
    }

    @Override
    public void initialize() {}


    @Override
    public void execute() {
        if(drivetrain.getAutoScoringState() == DrivetrainSubsystem.AutoScoringState.OFF) {
            drivetrain.driveRobotPedroFollower(strafe.getAsDouble() * 0.9, forward.getAsDouble() * 0.9, rotation.getAsDouble() * 0.6);
        } else {
            new SequentialCommandGroup(
                    new InstantCommand(() -> drivetrain.setAutoScoringState(DrivetrainSubsystem.AutoScoringState.ON)),
                    Commands.fastPath(drivetrain.follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevator)),
                    Commands.scoreSpeciman(elevator),
                    Commands.fastPath(drivetrain.follower, chain.getPath(1)).alongWith(Commands.retractThenIntake(elevator))
            );
        }
    }
}