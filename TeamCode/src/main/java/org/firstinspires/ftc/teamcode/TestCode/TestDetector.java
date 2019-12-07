package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name = "testDaDetector")
public class TestDetector extends LinearOpMode {

    SkystoneDetector detector;
    @Override
    public void runOpMode() throws InterruptedException {
        teamUtil.theOpMode = this;


        detector = new SkystoneDetector(telemetry, hardwareMap);
        detector.initDetector();

        detector.startTracking();



        waitForStart();

        while(opModeIsActive()){
            //detector.detect();
//            telemetry.addData("path: ", detector.detect());
            sleep(3000);
            detector.reportStoneInformation();
            telemetry.addData("path: ", detector.detect());

            telemetry.update();


        }

        detector.stopTracking();
    }
}
