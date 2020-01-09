package org.firstinspires.ftc.teamcode.CompetitionCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="AutoSkystonePathRed", group ="Red")


public class AutoSkystonePathRed extends LinearOpMode {

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
        teamUtil.telemetry.addLine("Initializing Op Mode");
        teamUtil.telemetry.update();
        initialize();
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

        robot.liftSystem.prepareToGrabNoWait(4000);
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

        if (path == 3) {
            robot.drive.decelerateInchesLeft(0.75, 43);
        } else if (path == 2) {
            robot.drive.decelerateInchesLeft(0.75, 38);
        } else if (path == 1) {
            robot.drive.stopMotors();
            teamUtil.log("path 1, stopping motors");
            return;
        }


        robot.liftSystem.prepareToGrabNoWait(4000);
//        robot.drive.rotateToZero();
        robot.drive.accelerateInchesBackward(0.5, 6, 2000);
        teamUtil.sleep(750);
        robot.drive.accelerateInchesForward(0.65, 10, 2000);
        robot.liftSystem.grabAndStow(4500);
        teamUtil.sleep(750);

        robot.drive.accelerateInchesBackward(0.6, 4.5, 2500);
//        robot.drive.rotateToZero();
        teamUtil.sleep(1000);
        robot.drive.accelerateToSpeedRight(0, 0.75);
        while (!robot.drive.bottomColor.isOnTape()) {
            robot.drive.driveRight(0.75);
        }
        robot.drive.decelerateInchesRight(0.75, 6);
        robot.liftSystem.grabber.slightlyOpenGrabber();
//        robot.drive.rotateToZero();
        teamUtil.sleep(750);
        while (!robot.drive.bottomColor.isOnTape()) {
            robot.drive.driveLeft(0.75);
        }
        robot.drive.stopMotors();





    }

//        robot.drive.setAllMotorsWithEncoder();
//        robot.drive.moveInchesForward(0.5, 10, 4500);
//        robot.drive.moveToDistance(robot.drive.frontLeftDistance, 11, 0.45, 4500);
//        robot.drive.rotateToZero();
//        teamUtil.sleep(2000);
//        while (!detector.reportStoneInformation() && teamUtil.theOpMode.opModeIsActive()){
//            robot.drive.driveLeft(0.3);
//            teamUtil.sleep(1000);
//            teamUtil.log("it does not work?" + detector.reportStoneInformation());
//        }
//        teamUtil.log("super cereal disclaimer");
//        robot.drive.stopMotors();




















//        robot.liftSystem.prepareToGrabNoWait(6000);
//        robot.drive.moveInchesForward(0.5, 12,5000);
//        int detected = detector.detectRed();
//        if (detected > 0) {
//            path = detected;
//        }
//        robot.drive.moveInchesForward(0.5, 6,5000);
//
//
//        if (path == 1) {
//
//            robot.drive.moveToDistance(robot.drive.frontLeftDistance, 8, Robot.AUTOINTAKE_POWER, 5000);
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
//            robot.liftSystem.grabAndStowNoWait("wide", 7000);
//
//            teamUtil.sleep(1000);
//
//            robot.drive.moveInchesBackward(0.5, 7,5000);
//
//
//            robot.drive.rotateToZero();
//
//
//
//
//            robot.drive.moveInchesRight(0.75, 93,15000);
//
//
//            robot.drive.rotateToZero();
//
//            robot.drive.moveInchesForward(0.5, 5, 3000);
//
//            robot.autoDropOffLeft(0, 9001);
//            teamUtil.sleep(500);
//
//            robot.drive.moveInchesBackward(0.5, 8, 4000);
//            robot.liftSystem.putAwayLiftSystemNoWait(6000);
//            robot.drive.moveInchesRight(0.5, 13, 5000);
//
//            robot.drive.rotateTo180();
//
//            //recall that robot turns backwards
//            robot.latch.latchMiddle();
//            robot.liftSystem.grabber.grabberStow();
//
//            robot.drive.moveInchesBackward(0.5, 8,4000 );
//            robot.drive.moveInchesBackward(0.25, 2, 2000);
//            robot.latch.latchDown();
//            robot.drive.moveInchesForward(0.5, 25, 7000);
//
//            robot.drive.moveToDistance(robot.drive.leftDistanceSensor, 2, Robot.AUTOINTAKE_POWER, 3000);
//
//            robot.drive.moveInchesForward(0.75, 7, 2000);
//            robot.latch.latchUp();
//            robot.drive.moveInchesRight(0.75, 35, 5000);
//            robot.drive.moveInchesBackward(0.5, 15, 3000);
//            robot.drive.moveInchesLeft(0.5, 25, 3000);
//            robot.drive.moveInchesRight(0.5, 45, 3000);
//
//
//
//
//
//
//            robot.liftSystem.hoverOverFoundationNoWait(0, Grabber.GrabberRotation.MIDDLE, 7000);
//
//
//
//        } else if (path == 2) {
//            robot.drive.moveToDistance(robot.drive.frontLeftDistance, 8, Robot.AUTOINTAKE_POWER, 5000);
//            robot.liftSystem.openGrabber();
//
//            robot.drive.rotateToZero();
//
//
//            robot.drive.moveInchesRight(0.5,5, 6000);
//
//            robot.drive.rotateToZero();
//
//            robot.drive.moveInchesForward(0.25, 8, 5000);
//            robot.drive.moveInchesBackward(0.25, 1.125, 3000);
//            robot.liftSystem.grabAndStowNoWait("wide", 7000);
//
//            teamUtil.sleep(1000);
//
//            robot.drive.moveInchesBackward(0.5, 7,5000);
//
//
//            robot.drive.rotateToZero();
//
//
//
//
//            robot.drive.moveInchesRight(0.75, 85,15000);
//
//
//            robot.drive.rotateToZero();
//
//            robot.drive.moveInchesForward(0.5, 5, 3000);
//
//            robot.autoDropOffLeft(0, 9001);
//            teamUtil.sleep(500);
//
//            robot.drive.moveInchesBackward(0.5, 8, 4000);
//            robot.liftSystem.putAwayLiftSystemNoWait(6000);
//            robot.drive.moveInchesRight(0.5, 13, 5000);
//
//            robot.drive.rotateTo180();
//
//            //recall that robot turns backwards
//            robot.latch.latchMiddle();
//            robot.liftSystem.grabber.grabberStow();
//
//            robot.drive.moveInchesBackward(0.5, 8,4000 );
//            robot.drive.moveInchesBackward(0.25, 2, 2000);
//            robot.latch.latchDown();
//            robot.drive.moveInchesForward(0.5, 25, 7000);
//
//            robot.drive.moveToDistance(robot.drive.leftDistanceSensor, 2, Robot.AUTOINTAKE_POWER, 3000);
//
//            robot.drive.moveInchesForward(0.75, 7, 2000);
//            robot.latch.latchUp();
//            robot.drive.moveInchesRight(0.75, 35, 5000);
//            robot.drive.moveInchesBackward(0.5, 15, 3000);
//            robot.drive.moveInchesLeft(0.5, 25, 3000);
//            robot.drive.moveInchesRight(0.5, 45, 3000);
//
//
//        } else if (path == 3) {
//            robot.drive.moveToDistance(robot.drive.frontLeftDistance, 8, Robot.AUTOINTAKE_POWER, 5000);
//            robot.liftSystem.openGrabber();
//
//            robot.drive.rotateToZero();
//
//
//            robot.drive.moveInchesRight(0.5,13, 6000);
//
//            robot.drive.rotateToZero();
//
//            robot.drive.moveInchesForward(0.25, 8, 5000);
//            robot.drive.moveInchesBackward(0.25, 1.125, 3000);
//            robot.liftSystem.grabAndStowNoWait("wide", 7000);
//
//            teamUtil.sleep(1000);
//
//            robot.drive.moveInchesBackward(0.5, 7,5000);
//
//
//            robot.drive.rotateToZero();
//
//
//
//
//            robot.drive.moveInchesRight(0.75, 77,15000);
//
//
//            robot.drive.rotateToZero();
//
//            robot.drive.moveInchesForward(0.5, 5, 3000);
//
//            robot.autoDropOffLeft(0, 9001);
//            teamUtil.sleep(500);
//
//            robot.drive.moveInchesBackward(0.5, 8, 4000);
//            robot.liftSystem.putAwayLiftSystemNoWait(6000);
//            robot.drive.moveInchesRight(0.5, 13, 5000);
//
//            robot.drive.rotateTo180();
//
//            //recall that robot turns backwards
//            robot.latch.latchMiddle();
//            robot.liftSystem.grabber.grabberStow();
//
//            robot.drive.moveInchesBackward(0.5, 8,4000 );
//            robot.drive.moveInchesBackward(0.25, 2, 2000);
//            robot.latch.latchDown();
//            robot.drive.moveInchesForward(0.5, 25, 7000);
//
//            robot.drive.moveToDistance(robot.drive.leftDistanceSensor, 2, Robot.AUTOINTAKE_POWER, 3000);
//
//            robot.drive.moveInchesForward(0.75, 7, 2000);
//            robot.latch.latchUp();
//            robot.drive.moveInchesRight(0.75, 35, 5000);
//            robot.drive.moveInchesBackward(0.5, 15, 3000);
//            robot.drive.moveInchesLeft(0.5, 25, 3000);
//            robot.drive.moveInchesRight(0.5, 45, 3000);
//
//        }

    }
