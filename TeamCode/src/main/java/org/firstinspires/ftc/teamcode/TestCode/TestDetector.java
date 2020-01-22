package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.runningVoteCount;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name = "testDaDetector")
//@Disabled

public class TestDetector extends LinearOpMode {

    SkystoneDetector detector;
    @Override
    public void runOpMode() throws InterruptedException {
        teamUtil.theOpMode = this;
        teamUtil.inInitialization = true;
        teamUtil.log("starting OpMode");


        detector = new SkystoneDetector(telemetry, hardwareMap);
        detector.initDetector();

        telemetry.addLine("initialized");
        telemetry.update();


        // Start looking at Skystones before the start...
        detector.activateDetector();
        telemetry.addLine("Starting to Detect");
        telemetry.update();
        //sleep(5000);

        runningVoteCount redCount = new runningVoteCount(3000);
        runningVoteCount blueCount = new runningVoteCount(3000);
        while (!opModeIsActive() && !isStopRequested()) {
            sleep(200);
            redCount.vote(detector.detectRed());
            blueCount.vote(detector.detectBlue());
            int redTotals[] = redCount.getTotals();
            int blueTotals[] = blueCount.getTotals();
            teamUtil.log("RED PATH: "+redCount.getWinner()+ " Totals:"+redTotals[1]+"/"+redTotals[2]+"/"+redTotals[3]
                          + "BLUE PATH: "+blueCount.getWinner()+ " Totals:"+blueTotals[1]+"/"+blueTotals[2]+"/"+blueTotals[3]);
        }


        //waitForStart();

        while(opModeIsActive()){
            //detector.detectRed();
//            telemetry.addData("path: ", detector.detectRed());
            ///sleep(3000);
            detector.reportStoneInformation();
            redCount.vote(detector.detectRed());
            blueCount.vote(detector.detectBlue());

            telemetry.addData("path: ", detector.detectRed());
            telemetry.addData("path: ", detector.detectBlue());

            telemetry.update();


        }

        detector.shutdownDector();
    }
}
