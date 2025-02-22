package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utilities.constants.Constants;

import java.util.List;

@TeleOp
public class BlueTeleop extends OpMode {
    private MecanumDrive drivetrain;
    private ElevatorSubsystem elevator;
    private Limelight3A limelight3A;
    private boolean gamepadApressed;
    @Override
    public void init() {
        drivetrain = new MecanumDrive(hardwareMap);
        elevator = new ElevatorSubsystem(hardwareMap, telemetry);

        limelight3A = hardwareMap.get(Limelight3A.class, "Limelight");
        telemetry.setMsTransmissionInterval(11);

        gamepadApressed = false;

        limelight3A.pipelineSwitch(0);
        limelight3A.start();
    }

    @Override
    public void loop() {
        //robotOrientation = imu.getRobotYawPitchRollAngles();
        //double yaw = robotOrientation.getYaw(AngleUnit.DEGREES);

        drivetrain.driveRobotCentric(gamepad1.left_stick_x * 0.75, -gamepad1.left_stick_y * 0.75, gamepad1.right_stick_x * 0.75);

        LLResult result = limelight3A.getLatestResult();

        if (result != null) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            //telemetry.addData("LL Latency", captureLatency + targetingLatency);
            //telemetry.addData("Parse Latency", parseLatency);
            //telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            if (result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();

                telemetry.addData("tx", tx);
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", ty);
                telemetry.addData("tync", result.getTyNC());

                telemetry.addData("Botpose", botpose.toString());

                // Access color results
                /*
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
                */

                if(gamepad1.a || gamepadApressed) {
                    gamepadApressed = true;

                    double rotation = tx * 0.05;
                    double forward = ty * 0.09;


                    if(rotation < 0.012 && forward < 0.012) {
                        rotation = 0;
                        forward = 0;

                        gamepadApressed = false;
                    }



                    drivetrain.driveRobotCentric(0.0, forward, rotation);
                    telemetry.addData("forward", forward);
                    telemetry.addData("rotation", rotation);
                }
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }
    }
}
