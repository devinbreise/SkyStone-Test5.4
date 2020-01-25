package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.runningVoteCount;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name = "testDaDetector")
//@Disabled

public class TestDetector extends LinearOpMode {

    SkystoneDetector detector;
    @Override
    public void runOpMode() throws InterruptedException {
        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;

        if(teamUtil.alliance == teamUtil.Alliance.RED){
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_RED);
        } else {
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_BLUE);
        }
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

        runningVoteCount voteCount = new runningVoteCount(3000);
        while (!opModeIsActive() && !isStopRequested()) {
            sleep(200);
            int vote = detector.detectRed();
            //int vote = detector.detectBlue();
            if (vote > 0) voteCount.vote(vote);
            int[] totals = voteCount.getTotals();
            teamUtil.log("PATH: "+voteCount.getWinner(3)+ " Totals:"+totals[1]+"/"+totals[2]+"/"+totals[3]);
            if (totals[1] > 0 && totals[2]==0 && totals[3] == 0) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED_PATH_1);
                teamUtil.log( "PATH: "+1);
            } else if (totals[3] > totals[2]*2) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED_PATH_3);
                teamUtil.log( "PATH: "+3);
            } else {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED_PATH_2);
                teamUtil.log( "PATH: "+2);
            }
        }


        //waitForStart();

        while(opModeIsActive()){
            //detector.detectRed();
//            telemetry.addData("path: ", detector.detectRed());
            ///sleep(3000);
            detector.reportStoneInformation();
            telemetry.addData("path: ", detector.detectRed());
            telemetry.addData("path: ", detector.detectBlue());

            telemetry.update();


        }

        detector.shutdownDector();
    }
}
