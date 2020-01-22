package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name = "AutoFoundationPathBlue", group = "Blue")
public class AutoFoundationPathBlue extends LinearOpMode {

    Robot robot;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.BLUE;

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

        teamUtil.telemetry.addLine("Ready to Start");
        teamUtil.telemetry.update();
        waitForStart();


        robot.foundation();
//    public void initialize() {
//        teamUtil.init(this);
//        teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_BLUE);
//
//        robot = new Robot(this);
//        robot.init(true);
//        teamUtil.theBlinkin.setSignal(Blinkin.Signals.BLUE_AUTO);
//
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        initialize();
//        teamUtil.telemetry.addLine("Initializing Op Mode");
//        teamUtil.telemetry.update();
//
//        robot.latch.latchUp();
//
//        teamUtil.telemetry.addLine("Ready to Start");
//        teamUtil.telemetry.update();
//        waitForStart();
//
//
//        teamUtil.telemetry.addLine("Ready to Start");
//        teamUtil.telemetry.update();
//        waitForStart();
//
//
//        robot.drive.moveInchesRight(0.5, 11, 4000);
//        robot.drive.moveInchesBackward(0.5, 32, 5000);
//        robot.latch.latchDown();
//        teamUtil.pause(750);
//        robot.drive.rotateToZero();
//
//
//
//        robot.latch.latchUp();
//        teamUtil.sleep(1000);
//
//        while (!robot.drive.bottomColor.isOnTape()) {
//            robot.drive.driveLeft(0.6);
////            if(robot.drive.rightDistanceSensor.getDistance() < 4 && robot.drive.rightDistanceSensor.getDistance() > 0){
////                robot.drive.accelerateInchesBackward(.6, 15, 3500);
////                teamUtil.log("saw the robot");
////            }
//        }
//
////        robot.drive.moveInchesBackward(0.5,20,5000);
////        robot.drive.moveInchesRight(0.5, 24,5000);
////        robot.drive.moveInchesForward(0.5, 25,5000);
////        robot.drive.moveInchesLeft(0.5, 32,5000);


    }
}
