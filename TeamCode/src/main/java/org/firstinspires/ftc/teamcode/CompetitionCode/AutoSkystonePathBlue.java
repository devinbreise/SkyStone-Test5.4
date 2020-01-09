package org.firstinspires.ftc.teamcode.CompetitionCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.SkystoneDetector;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="AutoSkystonePathBlue", group ="Blue")

@Disabled
public class AutoSkystonePathBlue extends LinearOpMode {

    Robot robot;
    boolean isSkystone;
    SkystoneDetector detector;
    int PATH = 1;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_RED);

        robot = new Robot(this);
        robot.init(true);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.BLUE_AUTO);

    }


    @Override
    public void runOpMode() throws InterruptedException {
        teamUtil.telemetry.addLine("Initializing Op Mode");
        teamUtil.telemetry.update();
        initialize();
        robot.latch.latchUp();
     //   detector.initDetector();

   //     detector.startTracking();
        teamUtil.telemetry.addLine("Ready to Start");
        teamUtil.telemetry.update();
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
            robot.liftSystem.grabAndStow(7000);
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
//            liftSystem.elevatorDown();
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