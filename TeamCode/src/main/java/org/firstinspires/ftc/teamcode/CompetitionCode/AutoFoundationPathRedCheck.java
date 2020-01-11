package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="AutoFoundationPathRedCheck", group ="Competition")
@Disabled

public class AutoFoundationPathRedCheck extends LinearOpMode {

    Robot robot;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_RED);

        robot = new Robot(this);
        robot.init(true);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED_AUTO);

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
