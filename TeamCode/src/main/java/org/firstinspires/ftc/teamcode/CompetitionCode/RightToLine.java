package org.firstinspires.ftc.teamcode.CompetitionCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name="RightToLine", group ="Competition")
public class RightToLine extends LinearOpMode {

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
            robot.drive.driveRight(0.35);
        }
        robot.drive.stopMotors();

    }
}
