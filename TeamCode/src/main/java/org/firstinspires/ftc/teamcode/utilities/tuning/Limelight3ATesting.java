package org.firstinspires.ftc.teamcode.utilities.tuning;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utilities.PIDFController;

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

    private static final double kPtx = 0.05;
    private static final double kPStx = 0.045;
    private static final double kDtx = 0.002;

    private static final double kPty = 0.05;
    private static final double kPSty = 0.045;
    private static final double kDty = 0.002;

    public static double target = 0;
    private double lastErrorTX, errorTX, tx = 0;
    private double lastErrorTY, errorTY, ty = 0;
    private double correctionTX, correctionTY;

    double[][] matriz = new double[6][6];
    int matrixIndex = 0;
    double sumErrorTX = 0; // Cumulative sum used to calculate the average of error
    double media;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        telemetry.setMsTransmissionInterval(11);
        drivetrainSubsystem = new DrivetrainSubsystem(hardwareMap, telemetry);

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        safetyTimer.reset(); // Start Maximum Alignment Timer
        aligned = false;
        targetSet = true; // Start timer when
    }

    @Override
    public void loop() {
        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);
        telemetry.addData("CorrectionTx", correctionTX);
        telemetry.addData("CorrectionTy", correctionTY);
        telemetry.addData("Error TX", errorTX);
        telemetry.addData("Error TY", errorTY);
        telemetry.update();

        LLResult result = limelight.getLatestResult();

        if (result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();

            errorTX = target + tx; // 1.5
            errorTY = target + ty;

            // Updating matrix and recalculate the sum and average
            int row = matrixIndex % 6;
            int col = matrixIndex / 6 % 6;

            sumErrorTX -= matriz[row][col]; // Remove old values
            matriz[row][col] = errorTX;     // Update the value in the array
            sumErrorTX += errorTX;          // Add the new value in the sum
            matrixIndex++;

            media = sumErrorTX / 36.0; // Calculating average

            double derivativeTX = errorTX - lastErrorTX;
            double derivativeTY = errorTY - lastErrorTY;
            lastErrorTX = errorTX;
            lastErrorTY = errorTY;

            if (errorTX < 2 && errorTX > -2) {
                correctionTX = (kPStx * errorTX) + (kDtx * derivativeTX);
            } else {
                correctionTX = (kPtx * errorTX) + (kDtx * derivativeTX);
            }

            if (errorTY < 2 && errorTY > -2) {
                correctionTY = (kPSty * errorTY) + (kDty * derivativeTY);
            } else {
                correctionTY = (kPty * errorTY) + (kDty * derivativeTY);
            }

            drivetrainSubsystem.driveRobotCentric(0, correctionTY * 0.7, correctionTX * 0.7);
        } else {
            telemetry.addData("No target found", "");
        }
    }
}
