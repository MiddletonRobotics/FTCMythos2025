package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.autonomous.paths.Paths;
import org.firstinspires.ftc.teamcode.commands.AutoIntakingThenTransfer;
import org.firstinspires.ftc.teamcode.commands.AutoScoringSpecimans;
import org.firstinspires.ftc.teamcode.commands.IntakeSample;
import org.firstinspires.ftc.teamcode.commands.IntakeSmapleThenRetract;
import org.firstinspires.ftc.teamcode.commands.PrepareToIntake;
import org.firstinspires.ftc.teamcode.commands.RobotDriveFollower;
import org.firstinspires.ftc.teamcode.commands.RobotOrientedDrive;
import org.firstinspires.ftc.teamcode.commands.ScoreSpecimanThenRetract;
import org.firstinspires.ftc.teamcode.commands.UninterruptableCommand;

import org.firstinspires.ftc.teamcode.commands.Commands;
import org.firstinspires.ftc.teamcode.commands.FieldOrientedDrive;
import org.firstinspires.ftc.teamcode.commands.IntakeFromWall;
import org.firstinspires.ftc.teamcode.commands.PrepareBucket;
import org.firstinspires.ftc.teamcode.commands.PrepareSpeciman;
import org.firstinspires.ftc.teamcode.commands.RetractElevator;
import org.firstinspires.ftc.teamcode.commands.ScoreBucketThenRetract;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LEDSubsystem;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="RobotController")
public class RobotController extends CommandOpMode {
    private DrivetrainSubsystem drivetrain;
    private IntakeSubsystem intake;
    private ElevatorSubsystem elevator;
    private LEDSubsystem led;
    private Limelight3A limelight;
    private IMU imu;

    public PathChain chain;
    private PathBuilder builder;

    private GamepadEx driverController, operatorController;
    private GamepadButton intakeSpecimanDriver,intakeDownDriver, outtakeClaw, intakeClaw, intakeDown, manualIntake, intakeRotate,intakeRotate2, autoTransfer, prepareSpeciman, scoreSpeciman, intakeSpeciman, sampleScoring, retractElevator, drivingToggle, extendSlides, autoScore, endScore, autoIntaking;

    private FtcDashboard dashboard;
    private List<Action> runningActions;

    @Override
    public void initialize() {
        drivetrain = new DrivetrainSubsystem(hardwareMap, telemetry);
        elevator = new ElevatorSubsystem(hardwareMap, telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        led = new LEDSubsystem(hardwareMap, telemetry);
        limelight.pipelineSwitch(0);
        limelight.start();

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        builder = new PathBuilder();
        chain = builder.addPath(
                new BezierLine(
                        new Point(
                                drivetrain.follower.getPose().getX(),
                                drivetrain.follower.getPose().getY(),
                                Point.CARTESIAN
                        ),
                        new Point(31.000, 80.000, Point.CARTESIAN)
                )
        ).setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierLine(
                                new Point(35.000,  80.000,  Point.CARTESIAN),
                                new Point(5.000, 36.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();

        driverController = new GamepadEx(gamepad1);
        operatorController = new GamepadEx(gamepad2);

        extendSlides = new GamepadButton(driverController, GamepadKeys.Button.A);
        autoTransfer = new GamepadButton(driverController, GamepadKeys.Button.B);
        outtakeClaw = new GamepadButton(driverController, GamepadKeys.Button.X);
        manualIntake = new GamepadButton(driverController, GamepadKeys.Button.Y);
        prepareSpeciman = new GamepadButton(driverController, GamepadKeys.Button.LEFT_BUMPER);
        scoreSpeciman = new GamepadButton(driverController, GamepadKeys.Button.RIGHT_BUMPER);
        autoScore = new GamepadButton(driverController, GamepadKeys.Button.START);
        endScore = new GamepadButton(driverController, GamepadKeys.Button.BACK);

        intakeClaw = new GamepadButton(operatorController, GamepadKeys.Button.A);
        autoIntaking = new GamepadButton(operatorController, GamepadKeys.Button.B);
        intakeDown = new GamepadButton(operatorController, GamepadKeys.Button.X);
        intakeSpeciman = new GamepadButton(operatorController, GamepadKeys.Button.Y);
        sampleScoring = new GamepadButton(operatorController, GamepadKeys.Button.RIGHT_BUMPER);
        retractElevator = new GamepadButton(operatorController, GamepadKeys.Button.LEFT_BUMPER);
        intakeRotate = new GamepadButton(operatorController, GamepadKeys.Button.DPAD_LEFT);
        intakeRotate2 = new GamepadButton(operatorController, GamepadKeys.Button.DPAD_RIGHT);

        autoTransfer.whenPressed(new IntakeSmapleThenRetract(elevator, intake, led));
        manualIntake.whenPressed(new IntakeSample(elevator, intake, led));
        outtakeClaw.whenPressed(
                new InstantCommand((() -> elevator.manipulatorToPosition(
                        elevator.getArmState(),
                        elevator.getWristState(),
                        ElevatorSubsystem.ClawState.OPEN_CLAW
                )), elevator)).whenReleased(new InstantCommand((() -> elevator.manipulatorToPosition(
                        elevator.getArmState(),
                        elevator.getWristState(),
                        ElevatorSubsystem.ClawState.CLOSE_CLAW
                )), elevator)
        );

        intakeClaw.whenPressed(
                new InstantCommand((() -> intake.intakeToPosition(
                        intake.getExtensionState(),
                        intake.getArmState(),
                        intake.getWristState(),
                        IntakeSubsystem.ClawState.OPEN_CLAW
                )), intake)).whenReleased(new InstantCommand((() -> intake.intakeToPosition(
                        intake.getExtensionState(),
                        intake.getArmState(),
                        intake.getWristState(),
                        IntakeSubsystem.ClawState.CLOSE_CLAW
                )), intake)
        );

        extendSlides.whenPressed(new PrepareToIntake(intake, elevator));
        intakeDown.toggleWhenPressed(
                new InstantCommand((() -> intake.intakeToPosition(
                        intake.getExtensionState(),
                        IntakeSubsystem.ArmState.INTAKING,
                        intake.getWristState(),
                        IntakeSubsystem.ClawState.OPEN_CLAW
                )), intake), new InstantCommand((() -> intake.intakeToPosition(
                        intake.getExtensionState(),
                        IntakeSubsystem.ArmState.READY,
                        intake.getWristState(),
                        IntakeSubsystem.ClawState.CLOSE_CLAW
                )), intake));

        autoIntaking.whenPressed(new AutoIntakingThenTransfer(drivetrain, elevator, intake, led, limelight));


        intakeRotate.toggleWhenPressed(new InstantCommand((() -> intake.intakeToPosition(
                intake.getExtensionState(),
                intake.getArmState(),
                IntakeSubsystem.WristState.ANGLED_90,
                intake.getClawState()
        )), intake), new InstantCommand((() -> intake.intakeToPosition(
                intake.getExtensionState(),
                intake.getArmState(),
                IntakeSubsystem.WristState.NORMAL,
                intake.getClawState()
        ))));

        intakeRotate2.toggleWhenPressed(new InstantCommand((() -> intake.intakeToPosition(
                intake.getExtensionState(),
                intake.getArmState(),
                IntakeSubsystem.WristState.ANGLED_30,
                intake.getClawState()
        )), intake), new InstantCommand((() -> intake.intakeToPosition(
                intake.getExtensionState(),
                intake.getArmState(),
                IntakeSubsystem.WristState.ANGLED_60,
                intake.getClawState()
        ))));

        autoScore.whenPressed(new SequentialCommandGroup(
                Commands.fastPath(drivetrain.follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevator)),
                Commands.scoreSpeciman(elevator),
                Commands.fastPath(drivetrain.follower, chain.getPath(1)).alongWith(Commands.retractThenIntake(elevator)),
                Commands.sleep(50).andThen(Commands.closeClaw(elevator)),
                Commands.sleep(50).andThen(Commands.fastPath(drivetrain.follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevator))),
                Commands.scoreSpeciman(elevator),
                Commands.fastPath(drivetrain.follower, chain.getPath(1)).alongWith(Commands.retractThenIntake(elevator)),
                Commands.sleep(50).andThen(Commands.closeClaw(elevator)),
                Commands.sleep(50).andThen(Commands.fastPath(drivetrain.follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevator))),
                Commands.scoreSpeciman(elevator),
                Commands.fastPath(drivetrain.follower, chain.getPath(1)).alongWith(Commands.retractThenIntake(elevator)),
                Commands.sleep(50).andThen(Commands.closeClaw(elevator)),
                Commands.sleep(50).andThen(Commands.fastPath(drivetrain.follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevator))),
                Commands.scoreSpeciman(elevator),
                Commands.fastPath(drivetrain.follower, chain.getPath(1)).alongWith(Commands.retractThenIntake(elevator)),
                Commands.sleep(50).andThen(Commands.closeClaw(elevator)),
                Commands.sleep(50).andThen(Commands.fastPath(drivetrain.follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevator))),
                Commands.scoreSpeciman(elevator),
                Commands.fastPath(drivetrain.follower, chain.getPath(1)).alongWith(Commands.retractThenIntake(elevator)),
                Commands.sleep(50).andThen(Commands.closeClaw(elevator)),
                Commands.sleep(50).andThen(Commands.fastPath(drivetrain.follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevator))),
                Commands.scoreSpeciman(elevator),
                Commands.fastPath(drivetrain.follower, chain.getPath(1)).alongWith(Commands.retractThenIntake(elevator)),
                Commands.sleep(50).andThen(Commands.closeClaw(elevator)),
                Commands.sleep(50).andThen(Commands.fastPath(drivetrain.follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevator))),
                Commands.scoreSpeciman(elevator),
                Commands.fastPath(drivetrain.follower, chain.getPath(1)).alongWith(Commands.retractThenIntake(elevator)),
                Commands.sleep(50).andThen(Commands.closeClaw(elevator)),
                Commands.sleep(50).andThen(Commands.fastPath(drivetrain.follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevator))),
                Commands.scoreSpeciman(elevator),
                Commands.fastPath(drivetrain.follower, chain.getPath(1)).alongWith(Commands.retractThenIntake(elevator)),
                Commands.sleep(50).andThen(Commands.closeClaw(elevator)),
                Commands.sleep(50).andThen(Commands.fastPath(drivetrain.follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevator))),
                Commands.scoreSpeciman(elevator),
                Commands.fastPath(drivetrain.follower, chain.getPath(1)).alongWith(Commands.retractThenIntake(elevator)),
                Commands.sleep(50).andThen(Commands.closeClaw(elevator)),
                Commands.sleep(50).andThen(Commands.fastPath(drivetrain.follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevator))),
                Commands.scoreSpeciman(elevator),
                Commands.fastPath(drivetrain.follower, chain.getPath(1)).alongWith(Commands.retractThenIntake(elevator)),
                Commands.sleep(50).andThen(Commands.closeClaw(elevator)),
                Commands.sleep(50).andThen(Commands.fastPath(drivetrain.follower, chain.getPath(0)).alongWith(Commands.prepareSpeciman(elevator)))

        ));
        endScore.whenPressed(new InstantCommand(() -> drivetrain.follower.breakFollowing()).andThen(new IntakeSmapleThenRetract(elevator, intake, led).alongWith(new InstantCommand(() -> drivetrain.follower.startTeleopDrive()))));

        intakeSpeciman.whenPressed(new IntakeFromWall(elevator));
        prepareSpeciman.whenPressed(new PrepareSpeciman(elevator));
        scoreSpeciman.whenPressed(new ScoreSpecimanThenRetract(elevator));
        sampleScoring.whenPressed(new PrepareBucket(elevator));
        retractElevator.whenPressed(new UninterruptableCommand(new ConditionalCommand(
                new ScoreBucketThenRetract(elevator),
                new RetractElevator(elevator),
                () -> elevator.getViperPosition() > 3000
        )));

        register(drivetrain, elevator, intake);
        schedule(new RunCommand(telemetry::update), new RunCommand(() -> drivetrain.follower.update()));
        drivetrain.setDefaultCommand(new RobotDriveFollower(drivetrain, driverController::getLeftX, driverController::getLeftY, driverController::getRightX));
        //drivetrain.setDefaultCommand(new RobotOrientedDrive(drivetrain, driverController::getLeftX, driverController::getLeftY, driverController::getRightX));

        telemetry.update();
    }


    @Override
    public void runOpMode() {
        initialize();

        while (!opModeIsActive()) {
            elevator.onInit();
            intake.onInit();
        }

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }

        reset();
    }
}
