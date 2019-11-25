package org.firstinspires.ftc.teamcode.CompetitionCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Latch;
import org.firstinspires.ftc.teamcode.Assemblies.Lift;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="AutoSkystonePathRed")

@Disabled
public class AutoSkystonePathRed extends LinearOpMode {

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


        robot.moveInchesForward(0.5, 5);

    }
}