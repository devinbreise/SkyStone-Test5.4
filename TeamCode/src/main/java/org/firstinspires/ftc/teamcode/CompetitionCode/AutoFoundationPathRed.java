package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="AutoFoundationPathRed", group ="Red")

public class AutoFoundationPathRed extends LinearOpMode {

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
        teamUtil.telemetry.addLine("Initializing Op Mode");
        teamUtil.telemetry.update();
        initialize();
        robot.latch.latchUp();

        teamUtil.telemetry.addLine("Ready to Start");
        teamUtil.telemetry.update();
        waitForStart();



        robot.drive.moveInchesLeft(0.5, 11, 4000);

            robot.drive.moveInchesBackward(0.5,28,5000);
            robot.latch.latchDown();
            teamUtil.sleep(750);
            robot.foundationRed();
            robot.latch.latchUp();
        teamUtil.sleep(1000);
        robot.drive.moveInchesRight(0.5, 10, 2300);
        while(!robot.drive.bottomColor.isOnTape()){
            robot.drive.driveRight(0.6);
//            if(robot.drive.rightDistanceSensor.getDistance() < 4 && robot.drive.rightDistanceSensor.getDistance() > 0){
//                robot.drive.accelerateInchesBackward(.6, 15, 3500);
//                teamUtil.log("saw the robot");
//            }
        }
        robot.drive.stopMotors();

//TODO: added code




//            robot.drive.moveInchesBackward(0.5,20,5000);
//            robot.drive.moveInchesLeft(0.5, 24,5000);
//            robot.drive.moveInchesForward(0.5, 20,5000);
//            robot.drive.moveInchesRight(0.5, 32,5000);







    }
}
