package org.firstinspires.ftc.teamcode.CompetitionCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="AutoSkystonePathRed")


public class AutoSkystonePathRed extends LinearOpMode {

    Robot robot;
    boolean isSkystone;
    SkystoneDetector detector;
    int path = 1;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Op Mode");
        telemetry.update();

        detector = new SkystoneDetector(telemetry, hardwareMap);
        robot = new Robot(this);
        teamUtil.theOpMode = this;

        robot.init();
        robot.latch.latchUp();
        //   detector.initDetector();

        //     detector.startTracking();
        telemetry.addLine("Ready to Start");
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            sleep(200);
            int detected = detector.detect();
            if (detected > 0) {
                path = detected;
            }
        }

        robot.drive.moveInchesForward(0.5, 19,5000);

        if (path == 1) {
            robot.liftSystem.prepareToGrab(6000);
            robot.liftSystem.openGrabber();
            robot.drive.moveInchesLeft(0.5,1.5, 6000);

            robot.drive.rotateToHeading(0);

            robot.drive.moveInchesForward(0.5, 7,5000);


            robot.liftSystem.grabAndStow("wide", 7000);
            robot.drive.moveInchesBackward(0.5, 7,5000);

            robot.drive.rotateToHeading(0);

            robot.drive.moveInchesRight(0.75, 90,15000);
            robot.drive.rotateToHeading(0);
            robot.drive.moveToDistance(robot.drive.frontLeftDistance, 7, robot.AUTOINTAKE_POWER, 5000);
            robot.autoDropOff(9001);

            //robot.liftSystem.hoverOverFoundationNoWait(0, Grabber.GrabberRotation.MIDDLE, 7000);



        } else if (path == 2) {

        } else if (path == 3) {

        }

    }
        }