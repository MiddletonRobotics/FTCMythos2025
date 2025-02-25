package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autonomous.paths.Paths;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class AutoScoringSpecimans extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private PathChain chain;

    public AutoScoringSpecimans(DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        chain = Paths.fiveSpecimanAutoRenewed;

        addRequirements(drivetrainSubsystem, elevatorSubsystem);
    }

    @Override
    public void execute() {
        drivetrainSubsystem.follower.update();
        new PathFollowerFast(drivetrainSubsystem.follower, chain.getPath(0)).alongWith(new PrepareSpeciman(elevatorSubsystem));
        Commands.scoreSpeciman(elevatorSubsystem);
        Commands.fastPath(drivetrainSubsystem.follower, chain.getPath(1)).alongWith(Commands.retractThenIntake(elevatorSubsystem));
    }
}
