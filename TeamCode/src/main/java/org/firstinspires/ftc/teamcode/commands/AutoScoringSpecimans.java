package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.autonomous.paths.Paths;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;

public class AutoScoringSpecimans extends SequentialCommandGroup {
    private PathChain chain;

    public AutoScoringSpecimans(DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem) {
        chain = Paths.fiveSpecimanAutoRenewed;

        addCommands(
                new InstantCommand(() -> drivetrainSubsystem.setAutoScoringState(DrivetrainSubsystem.AutoScoringState.ON)),
                Commands.sleep(1000),
                Commands.fastPath(drivetrainSubsystem.follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                Commands.scoreSpeciman(elevatorSubsystem),
                Commands.fastPath(drivetrainSubsystem.follower, chain.getPath(1)).alongWith(Commands.retractThenIntake(elevatorSubsystem))
        );
    }
}
