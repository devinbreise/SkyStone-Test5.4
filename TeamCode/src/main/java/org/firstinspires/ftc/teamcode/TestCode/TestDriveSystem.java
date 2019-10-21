package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;

@TeleOp(name = "TestDriveSystem")
public class TestDriveSystem extends OpMode {

    public static final double DRIVE_POWER = 0.5;
    RobotDrive robot;

    @Override
    public void init() {
        robot = new RobotDrive(hardwareMap, telemetry);
        robot.initMotors();

    }

    @Override
    public void loop() {


            robot.driveJoyStick(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if(gamepad1.dpad_up){
                robot.driveForward(DRIVE_POWER);
            }
            if(gamepad1.dpad_down){
                robot.driveBackward(DRIVE_POWER);
            }
            if(gamepad1.dpad_left){
                robot.driveLeft(DRIVE_POWER);
            }
            if(gamepad1.dpad_right){
                robot.driveRight(DRIVE_POWER);
            }


            if(gamepad1.left_bumper){
                robot.rotateCCW(DRIVE_POWER);
            } else if(gamepad1.right_bumper){
                robot.rotateCW(DRIVE_POWER);
            }
        //add triggers for speed boosts

    }
}
