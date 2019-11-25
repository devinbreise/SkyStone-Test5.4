package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name = "TestEncoderMovement")
@Disabled
public class TestEncoderMovement extends LinearOpMode {


    RobotDrive robot;


    @Override
    public void runOpMode() throws InterruptedException {
        teamUtil.theOpMode = this;

        robot = new RobotDrive(hardwareMap, telemetry);
        robot.initDriveMotors();
        robot.initImu();

        robot.resetAllDriveEncoders();
        robot.setAllMotorsWithoutEncoder();



        robot.encoderTelemetry();
        robot.driveTelemetry();
        telemetry.update();

        waitForStart();




            robot.moveInchesLeft(0.5, 30);
            sleep(2000);
            robot.moveInchesRight(0.5, 30);

            //robot.moveInchesLeft(0.5, 30);

    teamUtil.log("BACKLEFT " + robot.getBackLeftMotorPos());
    teamUtil.log("BACKRIGHT " + robot.getBackRightMotorPos());
    teamUtil.log("FRONTLEFT " + robot.getFrontLeftMotorPos());
    teamUtil.log("FRONTRIGHT " + robot.getFrontRightMotorPos());



//            robot.encoderTelemetry();
//            robot.driveTelemetry();
//            telemetry.update();






        }




    }

