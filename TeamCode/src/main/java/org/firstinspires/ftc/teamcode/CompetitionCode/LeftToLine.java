package org.firstinspires.ftc.teamcode.CompetitionCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name="LeftToLine", group ="Competition")
public class LeftToLine extends LinearOpMode {

    Robot robot;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT);
        robot = new Robot(this);
        robot.init();
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.READY_TO_START);

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


        telemetry.addLine("Ready to Start");
        telemetry.update();
        waitForStart();

        while(!robot.drive.bottomColor.isOnTape()){
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.CURIOSITY);
            robot.drive.driveLeft(0.35);
        }
        robot.drive.stopMotors();

    }
}
