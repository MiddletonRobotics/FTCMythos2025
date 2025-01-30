package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;

import org.firstinspires.ftc.teamcode.commands.PathFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.PathFollowerFast;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;

public class Commands {
    public static Command sleep(long ms) { return new WaitCommand(ms); }
    public static Command sleepUntil(BooleanSupplier condition) { return new WaitUntilCommand(condition); }
    public static Command instant(Runnable toRun, Subsystem... requirements) { return new InstantCommand(toRun, requirements); }

    public static PathFollowerCommand followPath(Follower follower, PathChain path) { return new PathFollowerCommand(follower, path); }
    public static PathFollowerCommand followPath(Follower follower, Path path) { return new PathFollowerCommand(follower, path); }
    public static PathFollowerFast fastPath(Follower follower, Path path) { return new PathFollowerFast(follower, path); }
    public static PathFollowerFast fastPath(Follower follower, PathChain path) { return new PathFollowerFast(follower, path); }

    public static Command prepareBucket(ElevatorSubsystem elevatorSubsystem) { return new PrepareBucket(elevatorSubsystem); }
    public static Command prepareSpeciman(ElevatorSubsystem elevatorSubsystem) { return new PrepareSpeciman(elevatorSubsystem); }
    public static Command prepareSpecimanAuto(ElevatorSubsystem elevatorSubsystem) { return new PrepareSpecimanAuto(elevatorSubsystem); }
    public static Command scoreSpeciman(ElevatorSubsystem elevatorSubsystem) { return new ScoreSpeciman(elevatorSubsystem); }
    public static Command scoreBucket(ElevatorSubsystem elevatorSubsystem) { return  new ScoreBucket(elevatorSubsystem); }
    public static Command scoreBucketThenRetract(ElevatorSubsystem elevatorSubsystem) { return new ScoreBucketThenRetract(elevatorSubsystem); }
    public static Command scoreSpecimanThenRetract(ElevatorSubsystem elevatorSubsystem) { return new ScoreSpecimanThenRetract(elevatorSubsystem); }
    public static Command retractElevator(ElevatorSubsystem elevatorSubsystem) { return new RetractElevator(elevatorSubsystem); }
    public static Command retractThenIntake(ElevatorSubsystem elevatorSubsystem) { return new RetractThenIntake(elevatorSubsystem); }

    public static Command intakeFromWall(ElevatorSubsystem elevatorSubsystem) { return new IntakeFromWall(elevatorSubsystem); }
    public static Command intakeIn(IntakeSubsystem intakeSubsystem) { return new IntakeIn(intakeSubsystem); }
    public static Command prepareIntakeWithoutExtension(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) { return new PrepareIntakeWithoutExtension(intakeSubsystem, elevatorSubsystem); }
    public static Command IntakeDown(IntakeSubsystem intakeSubsystem) { return new IntakeDown(intakeSubsystem); }
    public static Command intakeTransferWithoutRetraction(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) { return new IntakeTransferWithoutRetraction(elevatorSubsystem, intakeSubsystem); }
    public static Command closeClaw(ElevatorSubsystem elevatorSubsystem) { return  new CloseClaw(elevatorSubsystem); }
    public static Command fullyCloseClaw(ElevatorSubsystem elevatorSubsystem) { return  new FullyCloseClaw(elevatorSubsystem); }
    public static Command RotateIntakeToPosition(IntakeSubsystem intakeSubsystem) { return new RotateIntakeToPosition(intakeSubsystem); }
}
