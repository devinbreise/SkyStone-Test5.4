package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;

@TeleOp(name = "TestEncoderMovement")
public class TestEncoderMovement extends LinearOpMode {


    RobotDrive robot;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotDrive(hardwareMap, telemetry);


        robot.initDriveMotors();
        robot.driveTelemetry();

        waitForStart();

        while(opModeIsActive()){
            robot.moveInchesForward(0.5, 5);
            robot.driveTelemetry();

        }




    }
}
