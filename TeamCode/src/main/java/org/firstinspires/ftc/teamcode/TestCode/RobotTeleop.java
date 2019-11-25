package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Latch;
import org.firstinspires.ftc.teamcode.Assemblies.Lift;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "RobotTeleop")
public class RobotTeleop extends OpMode {

    public static double MAX_POWER = 1;
    public static double DRIVE_POWER = 1;
    RobotDrive robot;
    Lift lift;
    Latch latch;
    int level = 0;
    TeamGamepad teamGamePad;
    Grabber grabber;

    @Override
    public void init() {
        robot = new RobotDrive(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
        latch = new Latch(hardwareMap, telemetry);
        teamGamePad = new TeamGamepad(this);
        grabber = new Grabber(hardwareMap, telemetry);

        robot.initDriveMotors();
        robot.initImu();

        lift.initLift();
        latch.initLatch();
        grabber.initGrabber();

        teamUtil.initPerf();
        robot.resetHeading();

    }

    @Override
    public void loop() {

        //robot.scaleMovement(MAX_POWER, DRIVE_POWER);
        telemetry.addData("Heading:", robot.getHeading());
///////////////////////////////////////////////////////////////////////
        teamGamePad.gamepadLoop();
        lift.liftLoop(lift);

///////////////////////////////////////////////////////////////////////
        //this code is for the drive
        robot.universalJoystick(gamepad1.left_stick_x,
                gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                robot.getHeading());

        if(gamepad1.left_stick_button && gamepad1.right_stick_button){
            robot.resetHeading();
        }

//////////////////////////////////////////////////////////////////////
        //this code is for the foundation latch
        if (gamepad1.right_bumper) {
            latch.latchDown();

        }else if (gamepad1.dpad_up) {
            latch.latchUp();
        }

        if(gamepad1.dpad_left){
            latch.latchMiddle();
        }
/////////////////////////////////////////////////////////////////////
        //this code is for the grabber on the lift system
        if(gamepad1.y){
            grabber.openGrabber();
        }else if (teamGamePad.gamepad2bBounced()) {
            grabber.closeGrabberToggle();
        }

/////////////////////////////////////////////////////////////////////
        // this code is for the lift base
//        if (gamepad1.dpad_up) {
////            lift.liftBaseUp();
////        } else if (gamepad1.dpad_down) {
////            lift.liftBaseDown();
////        } else lift.shutDownLiftBase();
////////////////////////////////////////////////////////////////////
        //this code is for the lift system
        if(teamGamePad.gamepad2RightBumperBounced()){
            lift.goToLevel(level);
            level++;
        }
        if(gamepad2.left_bumper){
            lift.goToBottom();

            level = 0;
        }
        if(gamepad2.y){
            lift.tensionLiftString();
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
        telemetry.addData("level:", level);
        teamUtil.trackPerf();

    }
}

