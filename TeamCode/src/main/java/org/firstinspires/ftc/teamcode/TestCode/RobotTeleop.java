package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Latch;
import org.firstinspires.ftc.teamcode.Assemblies.Lift;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;

@TeleOp(name = "RobotTeleop")
public class RobotTeleop extends OpMode {

    public static double MAX_POWER = 1;
    public static double DRIVE_POWER = 1;
    RobotDrive robot;
    Lift lift;
    Latch latch;

    Grabber grabber;

    @Override
    public void init() {
        robot = new RobotDrive(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        latch = new Latch(hardwareMap, telemetry);

        grabber = new Grabber(hardwareMap, telemetry);

        robot.initDriveMotors();
//        lift.initLift();
        latch.initLatch();
//        grabber.initGrabber();


    }

    @Override
    public void loop() {

        //robot.scaleMovement(MAX_POWER, DRIVE_POWER);

        robot.driveJoyStick(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

//        if (gamepad1.dpad_up) {
//            lift.increaseLiftPower();
//        }
//        if (gamepad1.dpad_down) {
//            lift.decreaseLiftPower();
//        }

//        if (gamepad1.dpad_left) {
//
//        }
//        if (gamepad1.dpad_right) {
//
//        }

        if (gamepad1.a) {
            latch.latchUp();

        }
        if(gamepad1.x){
            latch.latchDown();
        }

        //open
        if (gamepad1.b) {
            latch.toggleLatch();

        }
        if (gamepad1.y) {
            latch.latchMiddle();

        }



//        if (gamepad1.b) {
//            if (gamepad1.y) {
//                grabber.openGrabber();
//            }
//            grabber.closeGrabberToggle();
//        }


//        if (gamepad1.left_bumper) {
//            lift.liftUp();
//        } else if (gamepad1.right_bumper) {
//            lift.liftDown();
//        } else lift.shutDownLift();
        //add triggers for speed boosts




/*
        lift.liftTelemetry();
*/
        latch.latchTelemetry();
        robot.driveTelemetry();
        telemetry.update();


    }
}

