package org.firstinspires.ftc.teamcode.CompetitionCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="AutoSkystonePathBlueShort", group ="Competition")


public class AutoSkystonePathBlueShort extends LinearOpMode {

    Robot robot;
    boolean isSkystone;
    SkystoneDetector detector;
    int path = 1;

    public void initialize() {
        teamUtil.init(this);
        robot = new Robot(this);
        robot.init();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Op Mode");
        telemetry.update();
        initialize();
        robot.latch.latchUp();

        detector = new SkystoneDetector(telemetry, hardwareMap);
        detector.initDetector();
        detector.activateDetector();

        telemetry.addLine("Ready to Start");
        telemetry.update();

        // Start detecting but wait for start of match to move
        while (!opModeIsActive() && !isStopRequested()) {
            sleep(200);

            teamUtil.log("path: " + path);
            int detected = detector.detectBlue();
            if (detected > 0) {
                path = detected;
            }
        }
        detector.shutdownDector();

        robot.liftSystem.prepareToGrabNoWait(6000);
        robot.drive.moveInchesForward(0.5, 18,5000);



        if (path == 1) {

            robot.drive.moveToDistance(robot.drive.frontLeftDistance, 9, Robot.AUTOINTAKE_POWER, 5000);
            robot.liftSystem.openGrabber();

            robot.drive.rotateToZero();


            robot.drive.moveInchesRight(0.5,4, 6000);

            robot.drive.rotateToZero();

            robot.drive.moveInchesForward(0.25, 8, 5000);
            robot.drive.moveInchesBackward(0.25, 1.125, 3000);
            robot.liftSystem.grabAndStowNoWait("wide", 7000);

            teamUtil.sleep(1000);

            robot.drive.moveInchesBackward(0.5, 2.8,5000);


            robot.drive.rotateToZero();
            robot.drive.moveInchesLeft(0.75, 60, 6000);
            robot.liftSystem.hoverOverFoundation(0, Grabber.GrabberRotation.INSIDE, 5000);
            robot.liftSystem.grabber.openGrabber();
            teamUtil.sleep(750);
            robot.liftSystem.putAwayLiftSystem(5000);
            robot.drive.moveInchesRight(0.5, 9, 6000);





        } else if (path == 2) {
            robot.drive.moveToDistance(robot.drive.frontLeftDistance, 9, Robot.AUTOINTAKE_POWER, 5000);
            robot.liftSystem.openGrabber();

            robot.drive.rotateToZero();


            robot.drive.moveInchesLeft(0.5,3, 6000);

            robot.drive.rotateToZero();

            robot.drive.moveInchesForward(0.25, 8, 5000);
            robot.drive.moveInchesBackward(0.25, 1.125, 3000);
            robot.liftSystem.grabAndStowNoWait("wide", 7000);

            teamUtil.sleep(1000);

            robot.drive.moveInchesBackward(0.5, 4,5000);


            robot.drive.rotateToZero();

            teamUtil.sleep(1000);

            robot.drive.moveInchesLeft(0.75, 52, 6000);
            robot.liftSystem.hoverOverFoundation(0, Grabber.GrabberRotation.INSIDE, 5000);
            robot.liftSystem.grabber.openGrabber();
            teamUtil.sleep(750);
            robot.liftSystem.putAwayLiftSystem(5000);
            robot.drive.moveInchesRight(0.5, 9, 6000);



        } else if (path == 3) {
            robot.drive.moveToDistance(robot.drive.frontLeftDistance, 9, Robot.AUTOINTAKE_POWER, 5000);
            robot.liftSystem.openGrabber();

            robot.drive.rotateToZero();


            robot.drive.moveInchesLeft(0.5,13, 6000);

            robot.drive.rotateToZero();

            robot.drive.moveInchesForward(0.25, 8, 5000);
            robot.drive.moveInchesBackward(0.25, 1.125, 3000);
            robot.liftSystem.grabAndStowNoWait("wide", 7000);

            teamUtil.sleep(1000);

            robot.drive.moveInchesBackward(0.5, 4.5,5000);


            robot.drive.rotateToZero();
            teamUtil.sleep(2000);

            robot.drive.moveInchesLeft(0.75, 50, 6000);
            robot.liftSystem.hoverOverFoundation(0, Grabber.GrabberRotation.INSIDE, 5000);
            robot.liftSystem.grabber.openGrabber();
            teamUtil.sleep(750);
            robot.liftSystem.putAwayLiftSystem(5000);
            robot.drive.moveInchesRight(0.5, 9, 6000);





        }
        detector.shutdownDector();

    }
}