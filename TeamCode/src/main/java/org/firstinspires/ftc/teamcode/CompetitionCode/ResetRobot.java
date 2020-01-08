package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name="ResetRobot")

public class ResetRobot extends LinearOpMode {
    public static final double SCALE_DOWN_CONSTANT = 0.3;
    int level = 0;
    Grabber.GrabberRotation grabberRotation;
    TeamGamepad teamGamePad;

    Robot robot;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_RED);
        robot = new Robot(this);

        teamGamePad = new TeamGamepad(this);

        robot.init(true);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.READY_TO_START);
        teamUtil.initPerf();
        robot.latch.latchUp(); // move latches up at start of teleop

    }

    @Override
    public void runOpMode() throws InterruptedException {


        teamUtil.telemetry.addLine("Initializing Op Mode...please wait");
        teamUtil.telemetry.update();
        initialize();
        grabberRotation = Grabber.GrabberRotation.INSIDE;

        teamUtil.telemetry.addLine("Robot Reset");
        teamUtil.telemetry.update();
//        teamUtil.nukeRobot();
        waitForStart();
    }
}
