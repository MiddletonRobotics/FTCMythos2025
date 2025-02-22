package org.firstinspires.ftc.teamcode.utilities.tuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

import java.util.List;

@TeleOp
public class Limelight3ATesting extends OpMode {
    private Limelight3A limelight;
    private DrivetrainSubsystem drivetrainSubsystem;

    private boolean targetSet = false;
    private boolean aligned = false;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime safetyTimer = new ElapsedTime();
    private static final double MINIMUM_ALIGN_TIME = 1.0;
    private static final double MAX_ALIGN_TIME = 3.0;

    private static final double KP = 0.028;
    private static final double KPB = 0.072;
    private static final double KD = 0.0025;

    public static double target = 4;
    private double lastErrorTX = 0;
    private double correctionTX, correctionTY;

    double[][] matriz = new double[6][6];
    int matrixIndex = 0;
    double sumErrorTX = 0; // Cumulative sum used to calculate the average of error
    double media;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        drivetrainSubsystem = new DrivetrainSubsystem(hardwareMap, telemetry);

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void start() {
        safetyTimer.reset(); // Start Maximum Alignment Timer
        aligned = false;
        targetSet = true; // Start timer when
    }

    @Override
    public void loop() {
        int alignedCycles = 0;

        while (!aligned) { // Keep running until aligned
            LLResult result = limelight.getLatestResult();

            if (result == null && !result.isValid()) {
                System.out.println(" Maximum Alignment Time Reached");
                limelight.stop();
                drivetrainSubsystem.driveRobotCentric(0, 0, 0);
            }

            double tx = result.getTx();
            double errorTX = -target - tx; // 1.5

            // Updating matrix and recalculate the sum and average
            int row = matrixIndex % 6;
            int col = matrixIndex / 6 % 6;

            sumErrorTX -= matriz[row][col]; // Remove old values
            matriz[row][col] = errorTX;     // Update the value in the array
            sumErrorTX += errorTX;          // Add the new value in the sum
            matrixIndex++;

            media = sumErrorTX / 36.0; // Calculating average

            double derivativeTX = errorTX - lastErrorTX;
            lastErrorTX = errorTX;

            if (Math.abs(media) < 3) {
                correctionTX = (KPB * errorTX) + (KD * derivativeTX);
            } else {
                correctionTX = (KP * errorTX) + (KD * derivativeTX);
            }

            drivetrainSubsystem.driveRobotCentric(0, 0, correctionTX * 0.7);

            if (Math.abs(media) < 0.8 && timer.seconds() > MINIMUM_ALIGN_TIME) {
                aligned = true;
                limelight.stop();
                drivetrainSubsystem.driveRobotCentric(0, 0, 0);
            }

            telemetry.addData("tx", tx);
            telemetry.addData("media", Math.abs(media));
            telemetry.addData("CorrectionTx", correctionTX);
            telemetry.addData("Alignment Status", aligned);
            telemetry.addData("Error TX", errorTX);
        }
    }
}
