package org.firstinspires.ftc.teamcode.CompetitionCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name="GarageBotAutoLEFT", group ="Competition")
public class TimedLeftToLine extends LinearOpMode {

    Robot robot;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_RED);
        robot = new Robot(this);
        robot.init(true);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.READY_TO_START);

    }

    @Override
    public void runOpMode() throws InterruptedException {

        teamUtil.telemetry.addLine("Initializing Op Mode");
        teamUtil.telemetry.update();
        initialize();
        robot.latch.latchUp();

        teamUtil.telemetry.addLine("Ready to Start");
        teamUtil.telemetry.update();
        waitForStart();


        teamUtil.telemetry.addLine("Ready to Start");
        teamUtil.telemetry.update();
        waitForStart();

        teamUtil.pause(16000);

        while(!robot.drive.bottomColor.isOnTape()){
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.CURIOSITY);
            robot.drive.driveLeft(0.35);
        }
        robot.drive.stopMotors();

    }
}
