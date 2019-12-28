package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name = "testDaDetector")


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

        while (!opModeIsActive() && !isStopRequested()) {
            sleep(200);
            detector.detectRed();
        }


        //waitForStart();

        while(opModeIsActive()){
            //detector.detectRed();
//            telemetry.addData("path: ", detector.detectRed());
            sleep(3000);
            detector.reportStoneInformation();
            telemetry.addData("path: ", detector.detectRed());
            telemetry.addData("path: ", detector.detectBlue());

            telemetry.update();


        }

        detector.shutdownDector();
    }
}
