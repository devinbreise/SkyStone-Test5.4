package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.TeamGamepad;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;


@TeleOp(name = "TestDriveSystem")
@Disabled
public class TestDriveSystem extends LinearOpMode {

    // lift system code - should be in its own assembly class...

    public static double MAX_POWER = 1;
    public double DRIVE_POWER = 0.5;
    Robot robot;
    boolean wasTurning = false;
    double storedHeading;

    public void initialize() {
        teamUtil.init(this);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.INIT_RED);
        robot = new Robot(this);

//        teamGamePad = new TeamGamepad(this);

        robot.init(true);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.READY_TO_START);
        teamUtil.initPerf();
        robot.latch.latchUp(); // move latches up at start of teleop

    }
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        storedHeading = robot.drive.getHeading();
        waitForStart();

        while (opModeIsActive()) {



//            teamUtil.telemetry.addData("Is Skystone?", colorSensor.isSkystone(colorSensor.getReading()));

            if(Math.abs(gamepad1.right_stick_x) < 0.1 && wasTurning){
                storedHeading = robot.drive.getHeading();
            }
            robot.drive.universalJoystick(gamepad1.left_stick_x,
                    gamepad1.left_stick_y,
                    gamepad1.right_stick_x, 1,
                    robot.drive.getHeading(), storedHeading);


            if (gamepad1.dpad_up) {
                robot.drive.driveForward(DRIVE_POWER);
            }
            if (gamepad1.dpad_down) {
                robot.drive.driveBackward(DRIVE_POWER);
            }
            if (gamepad1.dpad_left) {
                robot.drive.driveLeft(DRIVE_POWER);
            }
            if (gamepad1.dpad_right) {
                robot.drive.driveRight(DRIVE_POWER);
            }

            if (gamepad1.a) {
                robot.drive.resetHeading();
            }
//            if(gamepad1.a){ // this is overlapping with the lift test controls below...
//                robot.testDriveSlow();
//            }


            if (gamepad1.left_bumper) {
                robot.drive.rotateLeft(DRIVE_POWER);
            } else if (gamepad1.right_bumper) {
                robot.drive.rotateRight(DRIVE_POWER);
            }


            if (gamepad2.right_bumper) {
                robot.drive.rotateToHeading180Left();
            }
            if(gamepad2.left_bumper){
                robot.drive.rotateToZero();
            }

            if (gamepad2.dpad_up) {
                robot.drive.accelerateInchesForward(1, 40, 7000);
            } else if (gamepad2.dpad_down) {
                robot.drive.accelerateInchesBackward(1, 40, 7000);
            } else if (gamepad2.dpad_left) {
                robot.drive.accelerateInchesLeft(1, 40, 7000);
            } else if (gamepad2.dpad_right) {
                robot.drive.accelerateInchesRight(1, 40, 7000);
            } else if (gamepad2.a) {
                robot.drive.stopMotors();
            }
            //add triggers for speed boosts

//            robot.driveTelemetry();
            teamUtil.telemetry.addData("heading:", robot.drive.getHeading());
            teamUtil.telemetry.addData("ColorSensor: ", robot.drive.bottomColor.getReading());
            teamUtil.telemetry.addData("ColorSensor BlueTape?: ", robot.drive.bottomColor.onBlue());
            teamUtil.telemetry.addData("ColorSensor RedTape?: ", robot.drive.bottomColor.onRed());

            robot.drive.distanceTelemetry();
            robot.drive.telemetryDriveEncoders();
            teamUtil.telemetry.update();


        }
    }
}


