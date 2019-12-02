package org.firstinspires.ftc.teamcode.CompetitionCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Latch;
import org.firstinspires.ftc.teamcode.Assemblies.Lift;
import org.firstinspires.ftc.teamcode.Assemblies.LiftSystem;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="AutoSkystonePathBlue")
@Disabled
public class AutoSkystonePathBlue extends LinearOpMode {

    RobotDrive robot;
    Latch latch;
    LiftSystem liftSystem;
    boolean isSkystone;
    int PATH = 1;



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotDrive(hardwareMap, telemetry);
        latch = new Latch(hardwareMap, telemetry);
        liftSystem = new LiftSystem(hardwareMap, telemetry);
        teamUtil.theOpMode = this;

        robot.initDriveMotors();
        robot.initImu();
        latch.initLatch();
        robot.initDistanceSensors();
        liftSystem.initLiftSystem();

        while (!opModeIsActive()) {
            robot.resetHeading();
            telemetry.addData("heading:", robot.getHeading());
            telemetry.update();
        }


        waitForStart();


        robot.moveInchesForward(0.5, 22);
        //check color sensor value to see if we got a skystone. IF YES: PATH ==1
//        if (isSkystone) {
//            PATH = 1;
//        }
//        robot.moveInchesLeft(0.5, 3);
//        //check color sensor vlaue to see if we got a skystone IF YES: PATH == 2 ... IF NO: PATH == 3
//        if (isSkystone) {
//            PATH = 2;
//        } else PATH = 3;


        if (PATH == 1) {
            liftSystem.prepareToGrab();
            liftSystem.openGrabber();

            robot.moveInchesForward(0.5, 7);
            liftSystem.grabAndStowNoWait("wide");
            robot.moveInchesBackward(0.5, 7);


//            robot.moveInchesRight(0.5, 45);
//            liftSystem.hoverOverFoundationNoWait(0, Grabber.GrabberRotation.MIDDLE);
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
//            robot.imuRotate(180); // TEST DIS AHHHHHHHHHHHHHH
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