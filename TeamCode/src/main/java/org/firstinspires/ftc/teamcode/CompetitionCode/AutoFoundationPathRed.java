package org.firstinspires.ftc.teamcode.CompetitionCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Latch;
import org.firstinspires.ftc.teamcode.Assemblies.Lift;
import org.firstinspires.ftc.teamcode.Assemblies.LiftSystem;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
@Autonomous(name="AutoFoundationPathRed")
public class AutoFoundationPathRed extends LinearOpMode {

    RobotDrive robot;
    Latch latch;

    LiftSystem liftSystem;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Initializing Op Mode");
        telemetry.update();

        robot = new RobotDrive(hardwareMap, telemetry);
        latch = new Latch(hardwareMap, telemetry);
        liftSystem = new LiftSystem(hardwareMap, telemetry);
        teamUtil.theOpMode = this;

        robot.initDriveMotors();
        robot.initImu();
        latch.initLatch();
        liftSystem.initLiftSystem();

        telemetry.addLine("Ready to Start");
        telemetry.update();
        waitForStart();

        while(!opModeIsActive()){
            robot.resetHeading();
            telemetry.addData("heading:", robot.getHeading());
            telemetry.update();
        }


        waitForStart();


            robot.moveInchesBackward(0.5,32,5000);
            latch.latchDown();
            robot.moveInchesForward(0.5,37,6000);
            latch.latchUp();
            robot.moveInchesRight(0.5, 12,5000);
            robot.moveInchesBackward(0.5,1, 5000);
            robot.moveInchesRight(0.5, 28,5000);
            robot.moveInchesBackward(0.5,20,5000);
            robot.moveInchesLeft(0.5, 24,5000);
            robot.moveInchesForward(0.5, 20,5000);
            robot.moveInchesRight(0.5, 32,5000);







    }
}
