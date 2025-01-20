package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.paths.Paths;
import org.firstinspires.ftc.teamcode.commands.Commands;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Autonomous(name="5SpecAuto", group = "Working")
public class SpecimanAutonomousV5 extends CommandOpMode {
    public PathChain chain;
    public Follower follower;

    private ElevatorSubsystem elevatorSubsystem;
    private IntakeSubsystem intakeSubsystem;

    @Override
    public void initialize() {
        this.elevatorSubsystem = new ElevatorSubsystem(hardwareMap, telemetry);
        this.intakeSubsystem = new IntakeSubsystem(hardwareMap, telemetry);
        this.follower = new Follower(hardwareMap);
        this.follower.setStartingPose(new Pose(0.5, 73.0, 0.0));
        this.chain = Paths.fiveSpecimanAutoRenewed;

        schedule(
                new RunCommand(follower::update),
                new SequentialCommandGroup(
                        new WaitUntilCommand(this::opModeIsActive),
                        Commands.intakeIn(intakeSubsystem),
                        Commands.fastPath(follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        Commands.scoreSpeciman(elevatorSubsystem),
                        Commands.fastPath(follower, chain.getPath(1)).alongWith(Commands.retractElevator(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(2)).alongWith(new InstantCommand(() -> elevatorSubsystem.viperMotor.setPower(0))),
                        Commands.fastPath(follower, chain.getPath(3)),
                        Commands.fastPath(follower, chain.getPath(4)),
                        Commands.fastPath(follower, chain.getPath(5)).alongWith(new InstantCommand(() -> elevatorSubsystem.viperMotor.setPower(0))),
                        Commands.fastPath(follower, chain.getPath(6)).alongWith(Commands.intakeFromWall(elevatorSubsystem)),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(7)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(8)).alongWith(Commands.retractThenIntake(elevatorSubsystem)),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(9)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(10)).alongWith(Commands.retractThenIntake(elevatorSubsystem)),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(11)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(12)).alongWith(Commands.retractThenIntake(elevatorSubsystem)),
                        Commands.sleep(50).andThen(Commands.closeClaw(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(13)).alongWith(Commands.prepareSpeciman(elevatorSubsystem)),
                        Commands.sleep(100),
                        (Commands.scoreSpeciman(elevatorSubsystem)),
                        Commands.fastPath(follower, chain.getPath(14)).alongWith(Commands.retractElevator(elevatorSubsystem))
                )
        );
    }

    @Override
    public void runOpMode() {
        initialize();

        while (!opModeIsActive()) {

        }

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
            telemetry.addData("Viper Motor Power", elevatorSubsystem.viperMotor.getPower());
            telemetry.addData("Follower Busy", follower.isBusy() );
        }

        reset();
    }
}
