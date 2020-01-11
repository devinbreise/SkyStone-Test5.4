package org.firstinspires.ftc.teamcode.CompetitionCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="AutoSkystonePathBlueShort", group ="Blue")


public class AutoSkystonePathBlueShort extends LinearOpMode {
    Robot robot;
    boolean isSkystone;
    SkystoneDetector detector;
    int path = 1;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_BLUE);

        robot = new Robot(this);
        robot.init(true);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.BLUE_AUTO);

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

            int detected = detector.detectBlue();
            if (detected > 0) {
                path = detected;
                teamUtil.log("path: " + path);
            }
        }
        detector.shutdownDector();

        robot.liftSystem.prepareToGrabNoWait(4000);
        if (path == 3) {
        } else if (path == 2) {
            robot.drive.moveInchesRight(0.35, 7, 2300);
        } else if (path == 1) {
            robot.drive.accelerateInchesRight(0.35, 11.5, 5000);
        }


        robot.drive.accelerateInchesForward(0.75, 20, 3400);
        robot.drive.moveInchesForward(0.3, 3, 1000);
        robot.liftSystem.grabAndStowNoWait(4500);

        teamUtil.sleep(750);
        robot.drive.accelerateInchesBackward(0.6, 7.8, 2500);
//        robot.drive.rotateToZero();
        robot.drive.accelerateToSpeedLeft(0, 0.75);
        while (!robot.drive.bottomColor.isOnTape()) {
            robot.drive.driveLeft(0.75);
        }
        robot.drive.decelerateInchesLeft(0.75, 18);
        robot.liftSystem.grabber.slightlyOpenGrabber();
//        robot.drive.rotateToZero();
        robot.drive.stopMotors();
        teamUtil.sleep(750);
        while (!robot.drive.bottomColor.isOnTape()) {
            robot.drive.driveRight(0.75);
        }
        robot.drive.moveInchesLeft(0.3, 2, 1500);
        robot.drive.stopMotors();


    }

//        detector.shutdownDector();
//
//        robot.liftSystem.prepareToGrabNoWait(6000);
//        robot.drive.moveInchesForward(0.5, 18,5000);
//
//
//
//        if (path == 1) {
//
//            robot.drive.moveToDistance(robot.drive.frontLeftDistance, 9, Robot.AUTOINTAKE_POWER, 5000);
//            robot.liftSystem.openGrabber();
//
//            robot.drive.rotateToZero();
//
//
//            robot.drive.moveInchesRight(0.5,4, 6000);
//
//            robot.drive.rotateToZero();
//
//            robot.drive.moveInchesForward(0.25, 8, 5000);
//            robot.drive.moveInchesBackward(0.25, 1.125, 3000);
//            robot.liftSystem.grabAndStowNoWait(7000);
//
//            teamUtil.sleep(1000);
//
//            robot.drive.moveInchesBackward(0.5, 2.8,5000);
//
//
//            robot.drive.rotateToZero();
//            robot.drive.moveInchesLeft(0.75, 60, 6000);
//            robot.liftSystem.hoverOverFoundation(0, Grabber.GrabberRotation.INSIDE, 5000);
//            robot.liftSystem.grabber.openGrabber();
//            teamUtil.sleep(750);
//            robot.liftSystem.putAwayLiftSystem(5000);
//            robot.drive.moveInchesRight(0.5, 9, 6000);
//
//
//
//
//
//        } else if (path == 2) {
//            robot.drive.moveToDistance(robot.drive.frontLeftDistance, 9, Robot.AUTOINTAKE_POWER, 5000);
//            robot.liftSystem.openGrabber();
//
//            robot.drive.rotateToZero();
//
//
//            robot.drive.moveInchesLeft(0.5,3, 6000);
//
//            robot.drive.rotateToZero();
//
//            robot.drive.moveInchesForward(0.25, 8, 5000);
//            robot.drive.moveInchesBackward(0.25, 1.125, 3000);
//            robot.liftSystem.grabAndStowNoWait(7000);
//
//            teamUtil.sleep(1000);
//
//            robot.drive.moveInchesBackward(0.5, 4,5000);
//
//
//            robot.drive.rotateToZero();
//
//            teamUtil.sleep(1000);
//
//            robot.drive.moveInchesLeft(0.75, 52, 6000);
//            robot.liftSystem.hoverOverFoundation(0, Grabber.GrabberRotation.INSIDE, 5000);
//            robot.liftSystem.grabber.openGrabber();
//            teamUtil.sleep(750);
//            robot.liftSystem.putAwayLiftSystem(5000);
//            robot.drive.moveInchesRight(0.5, 9, 6000);
//
//
//
//        } else if (path == 3) {
//            robot.drive.moveToDistance(robot.drive.frontLeftDistance, 9, Robot.AUTOINTAKE_POWER, 5000);
//            robot.liftSystem.openGrabber();
//
//            robot.drive.rotateToZero();
//
//
//            robot.drive.moveInchesLeft(0.5,13, 6000);
//
//            robot.drive.rotateToZero();
//
//            robot.drive.moveInchesForward(0.25, 8, 5000);
//            robot.drive.moveInchesBackward(0.25, 1.125, 3000);
//            robot.liftSystem.grabAndStowNoWait(7000);
//
//            teamUtil.sleep(1000);
//
//            robot.drive.moveInchesBackward(0.5, 4.5,5000);
//
//
//            robot.drive.rotateToZero();
//            teamUtil.sleep(2000);
//
//            robot.drive.moveInchesLeft(0.75, 50, 6000);
//            robot.liftSystem.hoverOverFoundation(0, Grabber.GrabberRotation.INSIDE, 5000);
//            robot.liftSystem.grabber.openGrabber();
//            teamUtil.sleep(750);
//            robot.liftSystem.putAwayLiftSystem(5000);
//            robot.drive.moveInchesRight(0.5, 9, 6000);
//
//
//
//
//
//        }
//        detector.shutdownDector();

    }
