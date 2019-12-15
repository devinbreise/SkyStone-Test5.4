package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "testRobot")
public class TestRobot extends LinearOpMode {
    private Robot robot;

    public void initialize(){
        robot = new Robot(this);
        robot.init();
    }

    @Override
    public void runOpMode(){
        teamUtil.inInitialization = true;
        initialize();
        waitForStart();
        teamUtil.inInitialization = false;
        while (opModeIsActive()) {
            if(gamepad1.right_bumper){
                robot.autoIntake(8000);
            }
            if(gamepad1.left_bumper){
                robot.autoDropOffLeft(0, 8000);
            }
            if(gamepad2.x){
                robot.drive.moveInchesRight(0.5, 36, 15000);
            }
            if(gamepad2.right_bumper){
                robot.drive.rotateToHeading180Right();
            }
            if(gamepad2.left_bumper){
                robot.drive.rotateToHeadingZeroLeft();
            }
            if(gamepad1.a){
                robot.drive.moveToPickUpDistance(5000);
            }
        }
    }

}
