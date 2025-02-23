package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorKLNavxMicro;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;

public class AutoIntakingThenTransfer extends SequentialCommandGroup {
    public AutoIntakingThenTransfer(DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem, Limelight3A limelight) {
        addCommands(
                new AutoAlignToSample(drivetrainSubsystem, limelight),
                new PrepareToIntake(intakeSubsystem, elevatorSubsystem),
                Commands.IntakeDown(intakeSubsystem),
                Commands.sleep(175).andThen(new IntakeSmapleThenRetract(elevatorSubsystem, intakeSubsystem, ledSubsystem))
        );
    }
}
