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

    private PIDFController txController = new PIDFController(0.0065, 0.0, 0.000, 0.0);
    private PIDFController tyController = new PIDFController(0.1, 0.0, 0.0025, 0.0);

    private static final double KP = 0.028;
    private static final double KPB = 0.1;
    private static final double KD = 0.0025;

    public static double target = 0;
    private double tx;
    private double lastErrorTX = 0;
    private double correctionTX, correctionTY;
    private boolean alignedCycleStarted = false;

    double[][] matriz = new double[6][6];
    int matrixIndex = 0;
    double sumErrorTX = 0; // Cumulative sum used to calculate the average of error
    double media;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
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
        limelight.start();
        safetyTimer.reset(); // Start Maximum Alignment Timer
        aligned = false;
        targetSet = true; // Start timer when
    }

    @Override
    public void loop() {
            LLResult result = limelight.getLatestResult();

            if (result == null && !result.isValid()) {
                System.out.println(" Maximum Alignment Time Reached");
            }

            tx = result.getTx();
            txController.setSetPoint(target);

            drivetrainSubsystem.driveRobotCentric(0, 0, txController.calculate(tx, target));


        telemetry.addData("tx", tx);
        telemetry.addData("CorrectionTx", correctionTX);
        telemetry.addData("Alignment Status", aligned);
        telemetry.addData("Error TX", txController.getPositionError());
    }
}
