package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.revHubIMUGyro;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name = "TestImuMovement")
public class TestImuMovement extends LinearOpMode {


    RobotDrive robot;



    @Override
    public void runOpMode() throws InterruptedException {
        teamUtil.theOpMode = this;
        robot = new RobotDrive(hardwareMap, telemetry);

        robot.initDriveMotors();
        robot.initImu();
        telemetry.addData("heading:", robot.getHeading());
        telemetry.update();


        while(!opModeIsActive()){
            robot.resetHeading();
            telemetry.addData("heading:", robot.getHeading());
            telemetry.update();
        }



//            if(gamepad1.left_bumper){
//                robot.rotateCW(0.1);
//            } else if(gamepad1.right_bumper){
//                robot.rotateCCW(0.1);
//            } else robot.stopMotors();

//            if(gamepad1.a){
//                robot.resetHeading();
//            }
//
//           telemetry.addData("Absoluteheading:", robot.getAbsoluteHeading());
//            telemetry.addData("heading:", robot.getHeading());
//
//           telemetry.addData("INIT", RobotDrive.INITIAL_HEADING);
//            telemetry.update();


            sleep(2000);
                robot.imuRotate(90, 5000);
//                teamUtil.log("heading: " + robot.getHeading());




    }
}
