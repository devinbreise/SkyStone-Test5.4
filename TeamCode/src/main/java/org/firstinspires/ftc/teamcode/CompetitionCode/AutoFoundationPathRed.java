package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Latch;
import org.firstinspires.ftc.teamcode.Assemblies.Lift;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="AutoFoundationPath")
public class AutoFoundationPath extends LinearOpMode {

    RobotDrive robot;
    Latch latch;
    Lift lift;
    Grabber grabber;



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotDrive(hardwareMap, telemetry);
        latch = new Latch(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        grabber = new Grabber(hardwareMap, telemetry);
        teamUtil.theOpMode = this;

        robot.initDriveMotors();
        robot.initImu();
        latch.initLatch();
        lift.initLift();
        grabber.initGrabber();

        while(!opModeIsActive()){
            robot.resetHeading();
            telemetry.addData("heading:", robot.getHeading());
            telemetry.update();
        }


        waitForStart();


            robot.moveInchesBackward(0.5,32);
            latch.latchDown();
            robot.moveInchesForward(0.5,37);
            latch.latchUp();
            robot.moveInchesRight(0.5, 48);
            teamUtil.log("Heading: " + robot.getHeading());







    }
}
