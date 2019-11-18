package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "TestEncoderMovement")
public class TestEncoderMovement extends LinearOpMode {


    RobotDrive robot;


    @Override
    public void runOpMode() throws InterruptedException {
        teamUtil.theOpMode = this;

        robot = new RobotDrive(hardwareMap, telemetry);
        robot.initDriveMotors();

        robot.resetAllDriveEncoders();
        robot.setAllMotorsWithoutEncoder();



        robot.encoderTelemetry();
        robot.driveTelemetry();
        telemetry.update();

        waitForStart();




//            robot.moveInchesForward(0.5, 20);
        robot.driveForward(0.5, 500 );
//    robot.driveForward(0.5);

    teamUtil.log("BACKLEFT " + robot.getBackLeftMotorPos());
    teamUtil.log("BACKRIGHT " + robot.getBackRightMotorPos());
    teamUtil.log("FRONTLEFT " + robot.getFrontLeftMotorPos());
    teamUtil.log("FRONTRIGHT " + robot.getFrontRightMotorPos());



//            robot.encoderTelemetry();
//            robot.driveTelemetry();
//            telemetry.update();






        }




    }

