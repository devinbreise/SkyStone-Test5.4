package org.firstinspires.ftc.teamcode.CompetitionCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="AutoSkystonePathRedShort", group ="Competition")


public class AutoSkystonePathRedShort extends LinearOpMode {

    Robot robot;
    boolean isSkystone;
    SkystoneDetector detector;
    int path = 1;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT);

        robot = new Robot(this);
        robot.init();
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED_AUTO);

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

            int detected = detector.detectRed();
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


            robot.drive.moveInchesLeft(0.5,3, 6000);

            robot.drive.rotateToZero();

            robot.drive.moveInchesForward(0.25, 8, 5000);
            robot.drive.moveInchesBackward(0.25, 1.125, 3000);
            robot.liftSystem.grabAndStowNoWait(7000);

            teamUtil.sleep(1000);

            robot.drive.moveInchesBackward(0.5, 2.5,5000);


            robot.drive.rotateToZero();
            robot.drive.moveInchesRight(0.75, 60, 6000);
            robot.liftSystem.hoverOverFoundation(0, Grabber.GrabberRotation.INSIDE, 5000);
            robot.liftSystem.grabber.openGrabber();
            teamUtil.sleep(750);
            robot.liftSystem.putAwayLiftSystem(5000);
            robot.drive.moveInchesForward(0.5, 2, 3000);
            robot.drive.moveInchesLeft(0.5, 10, 6000);





        } else if (path == 2) {
            robot.drive.moveToDistance(robot.drive.frontLeftDistance, 9, Robot.AUTOINTAKE_POWER, 5000);
            robot.liftSystem.openGrabber();

            robot.drive.rotateToZero();


            robot.drive.moveInchesRight(0.5,5, 6000);

            robot.drive.rotateToZero();

            // Right here we might be directly in front of a stone if tensor flow got faked out, in which case the skystone is
            // in position 3.  Here is a simple test that should detect that.
            if (robot.drive.frontmiddleColor.red() > 21) {
                telemetry.addLine("WARNING: driving path 2, but stone is in path 3...");
                telemetry.update();
            }
            robot.drive.moveInchesForward(0.25, 8, 5000);
            robot.drive.moveInchesBackward(0.25, 1.125, 3000);
            robot.liftSystem.grabAndStowNoWait(7000);

            teamUtil.sleep(1000);

            robot.drive.moveInchesBackward(0.5, 3.5,5000);


            robot.drive.rotateToZero();

            teamUtil.sleep(1000);

            robot.drive.moveInchesRight(0.75, 50, 6000);
            robot.liftSystem.hoverOverFoundation(0, Grabber.GrabberRotation.INSIDE, 5000);
            robot.liftSystem.grabber.openGrabber();
            teamUtil.sleep(750);
            robot.liftSystem.putAwayLiftSystem(5000);
            robot.drive.moveInchesLeft(0.5, 7.5, 6000);



        } else if (path == 3) {
            robot.drive.moveToDistance(robot.drive.frontLeftDistance, 9, Robot.AUTOINTAKE_POWER, 5000);
            robot.liftSystem.openGrabber();

            robot.drive.rotateToZero();


            robot.drive.moveInchesRight(0.5,13, 6000);

            robot.drive.rotateToZero();

            robot.drive.moveInchesForward(0.25, 8, 5000);
            robot.drive.moveInchesBackward(0.25, 1.125, 3000);
            robot.liftSystem.grabAndStowNoWait(7000);

            teamUtil.sleep(1000);

            robot.drive.moveInchesBackward(0.5, 3,5000);


            robot.drive.rotateToZero();
            teamUtil.sleep(2000);
            robot.drive.moveInchesRight(0.75, 43, 6000);
            robot.liftSystem.hoverOverFoundation(0, Grabber.GrabberRotation.INSIDE, 5000);
            robot.liftSystem.grabber.openGrabber();
            teamUtil.sleep(750);
            robot.liftSystem.putAwayLiftSystem(5000);
            robot.drive.moveInchesLeft(0.5, 9.5, 6000);





        }

    }
}