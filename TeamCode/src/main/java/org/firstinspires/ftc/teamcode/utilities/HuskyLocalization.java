package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name="HuskyLensLocalization")
public class HuskyLocalization extends OpMode {
    private final int READ_PERIOD = 1;
    private HuskyLens huskyLens;
    private Deadline rateLimit;

    @Override
    public void init() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();

        if(!huskyLens.knock()) {
            telemetry.addData("Problem communicating with: ", huskyLens.getDeviceName());
        } else {
            telemetry.addData("Hardware Connected. ", "Press Start to Continue");
        }

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.update();
    }

    @Override
    public void loop() {
        if (!rateLimit.hasExpired()) {
            rateLimit.reset();
        }

        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block Count: ", blocks.length);

        for(int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
        }
    }
}
