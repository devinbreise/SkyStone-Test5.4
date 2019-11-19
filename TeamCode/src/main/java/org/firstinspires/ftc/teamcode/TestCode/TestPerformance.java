package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "testPerformance")
public class TestPerformance extends OpMode {
    public static final double MAJOR_INCREMENT = 0.05;
    public static final double MINOR_INCREMENT = 0.01;
    public static double INITIAL_POS = .5;
    public double currentPosition = INITIAL_POS;

    private teamUtil util = new teamUtil();
    private teamUtil.sampleStats samples = util.new sampleStats();
    private teamUtil.sampleStats samples2 = util.new sampleStats();

    private long lastLoopCall = Long.MAX_VALUE;


    public void init() {
        samples.setSampleNumberWindow(50);
        samples2.setTimeWindow(1000);
    }

    public void loop() {
        if (lastLoopCall == Long.MAX_VALUE) {return;}
        else {
            long now = System.currentTimeMillis();;
            samples.addSample((double)(now - lastLoopCall));
            samples2.addSample((double)(now - lastLoopCall));
            lastLoopCall = now;
            teamUtil.log("# RA:"+samples.getRunningAverage()+" L:"+samples.getLow()+" H:"+samples.getHigh());
            teamUtil.log("T RA:"+samples.getRunningAverage()+" L:"+samples.getLow()+" H:"+samples.getHigh());
        }
    }
}
