package org.firstinspires.ftc.teamcode.autonomous.depricated;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Config
@Autonomous(name = "AutonomousCfg")
public class SpecimanAutonomousV1 extends LinearOpMode {
    /*
    public class Outtake {
        private Servo grabberServo, wristServo, leftArmServo, rightArmServo;

        public Outtake(HardwareMap hardwareMap) {
            grabberServo = hardwareMap.get(Servo.class, "outtakeClaw");
            wristServo = hardwareMap.get(Servo.class, "outtakeWrist");
            leftArmServo = hardwareMap.get(Servo.class, "leftouttakeArm");
            rightArmServo = hardwareMap.get(Servo.class, "rightouttakeArm");

            grabberServo.setDirection(Servo.Direction.REVERSE);
            wristServo.setDirection(Servo.Direction.FORWARD);
            leftArmServo.setDirection(Servo.Direction.REVERSE);
            rightArmServo.setDirection(Servo.Direction.FORWARD);
        }

        public class ScorePosition implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    grabberServo.setPosition(Constants.outtakeClawClosedPosition);
                    wristServo.setPosition(Constants.outtakeWristScorePosition);
                    leftArmServo.setPosition(Constants.leftouttakeArmScorePosition);
                    rightArmServo.setPosition(Constants.rightouttakeArmScorePosition);
                    initialized = true;
                }

                double pos = leftArmServo.getPosition();
                packet.put("liftPos", pos);
                if (pos >= 0.1) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action setScore() {
            return new ScorePosition();
        }

        public class OpenClaw implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    grabberServo.setPosition(Constants.outtakeClawOpenPosition);
                    initialized = true;
                }

                double pos = leftArmServo.getPosition();
                packet.put("liftPos", pos);
                if (pos >= 0.01) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action clawOpen() {
            return new OpenClaw();
        }
    }

    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "viperMotor");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class ViperDown implements Action {
            private boolean initialized = false;
            private double currentPosition = lift.getCurrentPosition();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action viperDown() {
            return new ViperDown();
        }

        public class ViperSpecimanReady implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1200.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftReadyPosition(){
            return new ViperSpecimanReady();
        }

        public class ViperSpecimanScore implements Action {
            private boolean initialized = false;
            private double currentPosition = lift.getCurrentPosition();

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2250.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftSpeciamnScore(){
            return new ViperSpecimanScore();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-64, -8, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Lift lift = new Lift(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);

        TrajectoryActionBuilder autonomous = drive.actionBuilder(initialPose)
                .afterTime(0.5, new ParallelAction(
                        outtake.setScore(),
                        lift.liftReadyPosition()
                ))
                .splineTo(new Vector2d(-33, 0), Math.toRadians(0))
                        .afterTime(1, new SequentialAction(
                                lift.liftSpeciamnScore(),
                                new SleepAction(1),
                                outtake.clawOpen()
                        ));

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(outtake.setScore());
        Actions.runBlocking(lift.liftReadyPosition());

        Actions.runBlocking(new SequentialAction(
                autonomous.build(),
                lift.liftSpeciamnScore(),
                outtake.clawOpen()
        ));

    }

     */

    private boolean aButtonPreviousState;
    private enum AutonomousType {
        NONE,
        SPECIMAN,
        BUCKET
    }

    private AutonomousType autonomousChooser = AutonomousType.SPECIMAN;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-64, -8, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ElevatorSubsystem elevator = new ElevatorSubsystem(hardwareMap, telemetry);
        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap, telemetry);

        /*

        TrajectoryActionBuilder speciman = drive.actionBuilder(initialPose)
                .afterTime(0.5, new ParallelAction(
                        elevator.prepareToScore(),
                        elevator.elevatorSpecimanReadyPosition(),
                        intake.retractIntake()
                ))
                .splineTo(new Vector2d(-33, 0), Math.toRadians(0))
                .afterTime(1, new SequentialAction(
                        elevator.elevatorSpecimanScorePosition(),
                        intake.transferPosition(),
                        new SleepAction(1),
                        elevator.openClaw()
                ));
        //                        .setTangent(0)
//                                .splineToConstantHeading(new Vector2d(-64, ))

         */

        waitForStart();
        if (isStopRequested()) return;

        telemetry.addData("Position X", drive.pose.position.x);
        telemetry.addData("Position Y", drive.pose.position.y);
        telemetry.addData("Heading", drive.pose.heading);

        Actions.runBlocking(new SequentialAction(
                //speciman.build()
        ));
    }
}
