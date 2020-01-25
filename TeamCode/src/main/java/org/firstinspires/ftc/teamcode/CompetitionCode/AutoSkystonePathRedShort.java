package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name = "RedSingleSkystone", group = "Red")
@Disabled
public class AutoSkystonePathRedShort extends LinearOpMode {

    Robot robot;
    boolean isSkystone;
    SkystoneDetector detector;
    int path = 1;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_RED);

        robot = new Robot(this);
        robot.init(true);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED_AUTO);

    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        teamUtil.telemetry.addLine("Initializing Op Mode");
        teamUtil.telemetry.update();
        robot.latch.latchUp();

        detector = new SkystoneDetector(telemetry, hardwareMap);
        detector.initDetector();
        detector.activateDetector();

        teamUtil.telemetry.addLine("Ready to Start");
        teamUtil.telemetry.update();

        // Start detecting but wait for start of match to move
        while (!opModeIsActive() && !isStopRequested()) {
            sleep(200);

            int detected = detector.detectRed();
            if (detected > 0) {
                path = detected;
            }
        }
        detector.shutdownDector();

        robot.liftSystem.prepareToGrabNoWait(4000, Grabber.GrabberRotation.INSIDE);
        if (path == 3) {
        } else if (path == 2) {
            robot.drive.moveInchesLeft(0.35, 7, 2300);
        } else if (path == 1) {
            robot.drive.moveInchesLeft(0.35, 15, 2300);
        }


        robot.drive.accelerateInchesForward(0.75, 20, 3400);
        robot.drive.moveInchesForward(0.3, 3, 1000);
        robot.liftSystem.grabAndStowNoWait(4500);

        teamUtil.sleep(750);
        robot.drive.accelerateInchesBackward(0.6, 7.8, 2500);
//        robot.drive.rotateToZero();
        robot.drive.accelerateToSpeedRight(0, 0.75);
        while (!robot.drive.bottomColor.isOnTape()) {
            robot.drive.driveRight(0.75);
        }
        robot.drive.decelerateInchesRight(0.75, 6);
        robot.liftSystem.grabber.slightlyOpenGrabber();
//        robot.drive.rotateToZero();
        robot.drive.stopMotors();
        teamUtil.sleep(750);
        while (!robot.drive.bottomColor.isOnTape()) {
            robot.drive.driveLeft(0.75);
        }
        robot.drive.stopMotors();





    }
}