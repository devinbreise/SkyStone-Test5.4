package org.firstinspires.ftc.teamcode.CompetitionCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Latch;
import org.firstinspires.ftc.teamcode.Assemblies.LiftSystem;
import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="AutoSkystonePathBlue")

public class AutoSkystonePathBlue extends LinearOpMode {

    Robot robot;
    boolean isSkystone;
    SkystoneDetector detector;
    int PATH = 1;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Op Mode");
        telemetry.update();

        robot = new Robot(this);
        teamUtil.theOpMode = this;

        robot.init();
        robot.latch.latchUp();
     //   detector.initDetector();

   //     detector.startTracking();
        telemetry.addLine("Ready to Start");
        telemetry.update();
        while(!opModeIsActive()){
            robot.drive.resetHeading();
        }


        waitForStart();


        robot.drive.moveInchesForward(0.5, 19,5000);

        if (PATH == 1) {
            robot.liftSystem.prepareToGrab(6000);
            robot.liftSystem.openGrabber();
            robot.drive.moveInchesLeft(0.5, 4, 5000);

            robot.drive.rotateToHeading(0);
            robot.drive.moveInchesForward(0.5, 8,5000);
            robot.liftSystem.grabAndStow("wide", 7000);
            robot.drive.moveInchesBackward(0.5, 7,5000);


            robot.drive.moveInchesRight(0.5, 45,5000);
            robot.liftSystem.hoverOverFoundationNoWait(0, Grabber.GrabberRotation.MIDDLE, 7000);
//
//            do{
//                robot.driveRight(0.5);
//            }while(robot.getDistanceInches(robot.frontLeftDistance) > 15 && teamUtil.theOpMode.opModeIsActive());
//            robot.stopMotors();
//
//            robot.moveInchesForward(0.5, 10);
//            liftSystem.drop();
//
//            robot.moveInchesBackward(0.5, 7);
//            liftSystem.liftDown();
//            robot.turn(180); // TEST DIS AHHHHHHHHHHHHHH
//            robot.moveInchesBackward(0.5, 10);
//            latch.latchDown();
//            sleep(2000);
//
//            robot.moveInchesForward(0.5, 30);
//            robot.moveInchesRight(0.5, 24);
//            robot.moveInchesBackward(0.5, 24);
//            robot.moveInchesRight(0.5, 10);
//









        } else if (PATH == 2) {

        } else if (PATH == 3) {

        }

    }


}