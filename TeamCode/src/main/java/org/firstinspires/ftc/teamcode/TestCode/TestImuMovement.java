package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.revHubIMUGyro;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "TestImuMovement")
public class TestImuMovement extends LinearOpMode {


    RobotDrive robot;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotDrive(hardwareMap, telemetry);

        robot.initDriveMotors();
        robot.initImu();
        telemetry.addData("heading:", robot.getHeading());
        telemetry.update();

        waitForStart();

        robot.resetHeading();

        while (opModeIsActive()) {
           telemetry.addData("Absoluteheading:", robot.getAbsoluteHeading());
            telemetry.addData("heading:", robot.getHeading());

           telemetry.addData("INIT", robot.INITIAL_HEADING);
            telemetry.update();

            sleep(5000);

            robot.imuRotateToAngle(0);
            teamUtil.log("heading: " + robot.getHeading());

            sleep(5000);

            robot.imuRotateToAngle(90);
        }


    }
}
