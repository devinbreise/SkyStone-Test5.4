package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name = "RedFoundation", group = "Red")

public class AutoFoundationPathRed extends LinearOpMode {

    Robot robot;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;
        if(teamUtil.alliance == teamUtil.Alliance.RED){
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_RED);
        } else {
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_BLUE);
        }

        robot = new Robot(this);
        robot.init(true);
        if(teamUtil.alliance == teamUtil.Alliance.RED) {
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED_AUTO);
        } else {
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.BLUE_AUTO);
        }
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


        robot.foundation();

    }
}
