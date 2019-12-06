package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;

@TeleOp(name = "testRobot")
public class TestRobot extends OpMode {
    private Robot robot;

    @Override
    public void init(){
        robot = new Robot(telemetry, hardwareMap);
        robot.init();
    }

    @Override
    public void loop(){
        if(gamepad1.right_bumper){
            robot.autoIntake(true);
        }
    }
}
