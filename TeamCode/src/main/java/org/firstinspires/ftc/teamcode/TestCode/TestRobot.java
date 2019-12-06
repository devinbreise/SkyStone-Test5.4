package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;

@TeleOp(name = "testRobot")
public class TestRobot extends LinearOpMode {
    private Robot robot;

    public void initialize(){
        robot = new Robot(this);
        robot.init();
    }

    @Override
    public void runOpMode(){
        initialize();
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.right_bumper){
                robot.autoIntake(true, 8000);
            }
        }
    }

}
