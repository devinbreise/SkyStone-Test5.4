package org.firstinspires.ftc.teamcode.CompetitionCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="DoubleSkystone", group ="ULTIMATEPOWER")


public class AutoSkystonePathRed extends LinearOpMode {

    Robot robot;
    boolean isSkystone;
    SkystoneDetector detector;


    public void initialize() {
        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;

        if(teamUtil.alliance == teamUtil.Alliance.RED){
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_RED);
        } else {
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_BLUE);
        }

        robot = new Robot(this);
        robot.init(true);
        if(teamUtil.alliance == teamUtil.Alliance.RED) {
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED_AUTO);
        } else {
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.BLUE_AUTO);
        }

    }


    @Override
    public void runOpMode() throws InterruptedException {


        initialize();
        teamUtil.telemetry.addLine("Initializing Op Mode");
        teamUtil.telemetry.update();
        robot.latch.latchUp();


        robot.doubleSkystone();




//
//        // Start detecting but wait for start of match to move
//        while (!opModeIsActive() && !isStopRequested()) {
//            sleep(200);
//
//            int detected = detector.detectRed();
//            if (detected > 0) {
//                path = detected;
//            }
//        }
//        detector.shutdownDector();
//
//        robot.liftSystem.prepareToGrabNoWait(4000);
//        if (path == 3) {
//        } else if (path == 2) {
//            robot.drive.moveInchesLeft(0.35, 7, 2300);
//        } else if (path == 1) {
//            robot.drive.moveInchesLeft(0.35, 15, 2300);
//        }
//
//
//        robot.drive.accelerateInchesForward(0.65, 20, 3400);
//        robot.drive.moveInchesForward(0.3, 3, 1000);
//        robot.liftSystem.grabAndStowNoWait(4500);
//
//        teamUtil.sleep(750);
//        robot.drive.accelerateInchesBackward(0.6, 9.3, 2500); //TODO: changed dis
//        robot.drive.rotateToZero();
//        robot.drive.accelerateToSpeedRight(0, 0.75);
//        while (!robot.drive.bottomColor.isOnTape()) {
//            robot.drive.driveRight(0.75);
//        }
//        robot.drive.decelerateInchesRight(0.75, 6);
//        robot.liftSystem.grabber.slightlyOpenGrabber();
//        robot.drive.rotateToZero();
//        teamUtil.sleep(750);
//        robot.drive.accelerateToSpeedLeft(0.35, 1);
//        while (!robot.drive.bottomColor.isOnTape()) {
//            robot.drive.driveLeft(1);
//        }
//
//        if (path == 3) {
//            robot.drive.decelerateInchesLeft(1, 44);
//        } else if (path == 2) {
//            robot.drive.decelerateInchesLeft(1, 50);
//        } else if (path == 1) {
//            robot.drive.stopMotors();
//            teamUtil.log("path 1, stopping motors");
//            return;
//        }
//
//
//        robot.liftSystem.prepareToGrabNoWait(4000);
//        robot.drive.rotateToZero();
//        robot.drive.moveToDistance(robot.drive.frontLeftDistance, 9, 0.45, 3000);
//        while(!robot.liftSystem.preparedToGrab){
//            teamUtil.sleep(100);
//        }
//        robot.drive.moveToDistance(robot.drive.frontLeftDistance, 6, 0.3, 3000);
//        robot.drive.accelerateInchesForward(0.65, 7, 2000);
//        robot.liftSystem.grabAndStowNoWait(4500);
//        teamUtil.sleep(750);
//
//        if(path == 3){
//            robot.drive.moveInchesBackward(0.35, 9, 3000); //TODO: this didn't work on path 3...? --> fixed it, awaiting next download 1/10/19 4:12 PM
//        } else if(path == 2){
//            robot.drive.moveInchesBackward(0.35, 10, 3000);
//        }
//        robot.drive.rotateToZero();
//        robot.drive.accelerateToSpeedRight(0, 1);
//        while (!robot.drive.bottomColor.isOnTape()) {
//            robot.drive.driveRight(1);
//        }
//        robot.drive.decelerateInchesRight(1, 6);
//        robot.liftSystem.grabber.slightlyOpenGrabber();
//        robot.drive.rotateToZero();
//        while (!robot.drive.bottomColor.isOnTape()) {
//            robot.drive.driveLeft(0.75);
//        }
//        robot.drive.stopMotors();
//




    }


    }
