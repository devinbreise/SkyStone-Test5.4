package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Latch;
import org.firstinspires.ftc.teamcode.Assemblies.Lift;
import org.firstinspires.ftc.teamcode.Assemblies.LiftSystem;
import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="AutoFoundationPathRedCheck", group ="Competition")
public class AutoFoundationPathRedCheck extends LinearOpMode {

    Robot robot;

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

        telemetry.addLine("Ready to Start");
        telemetry.update();
        waitForStart();



        robot.drive.moveInchesLeft(0.5, 11, 4000);

        robot.drive.moveInchesBackward(0.5,32,5000);
        robot.latch.latchDown();
        teamUtil.sleep(750);
        robot.drive.moveInchesForward(0.5,37,6000);
        robot.latch.latchUp();
        teamUtil.sleep(1000);

        robot.drive.moveInchesRight(0.5, 12,5000);
        robot.drive.moveInchesBackward(0.5,0.5, 5000);
        robot.drive.moveInchesRight(0.5, 28,5000);
        robot.drive.moveInchesBackward(0.5,20,5000);
        robot.drive.moveInchesLeft(0.5, 24,5000);
        robot.drive.moveInchesForward(0.5, 20,5000);
        robot.drive.moveInchesRight(0.5, 32,5000);







    }
}