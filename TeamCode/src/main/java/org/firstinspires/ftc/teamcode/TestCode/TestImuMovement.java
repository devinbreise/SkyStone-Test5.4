package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.revHubIMUGyro;

@TeleOp(name = "TestImuMovement")
public class TestImuMovement extends LinearOpMode {

    HardwareMap hardwareMap;
    Telemetry telemetry;

    revHubIMUGyro imu;
    RobotDrive robot;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotDrive(hardwareMap, telemetry);

        robot.initDriveMotors();
        robot.initImu();

        while (opModeIsActive()) {
            imu.getHeading();
        }


    }
}
