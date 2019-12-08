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
@Autonomous(name="AutoFoundationPathBlue")
public class AutoFoundationPathBlue extends LinearOpMode {

    Robot robot;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Initializing Op Mode");
        telemetry.update();

        robot = new Robot(this);

        teamUtil.theOpMode = this;

        robot.init();
        robot.latch.latchUp();

        telemetry.addLine("Ready to Start");
        telemetry.update();
        waitForStart();



        telemetry.addLine("Ready to Start");
        telemetry.update();
        waitForStart();


        robot.drive.moveInchesBackward(0.5,32,5000);
        robot.latch.latchDown();
        robot.drive.moveInchesForward(0.5,37,6000);
        robot.latch.latchUp();
        robot.drive.moveInchesLeft(0.5, 12,5000);
        robot.drive.moveInchesBackward(0.5,1,2000);
        robot.drive.moveInchesLeft(0.5, 28,5000);
        robot.drive.moveInchesBackward(0.5,20,5000);
        robot.drive.moveInchesRight(0.5, 24,5000);
        robot.drive.moveInchesForward(0.5, 25,5000);
        robot.drive.moveInchesLeft(0.5, 32,5000);







    }
}
