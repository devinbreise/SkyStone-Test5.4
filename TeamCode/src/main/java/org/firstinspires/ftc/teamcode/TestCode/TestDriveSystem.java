package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;


@TeleOp(name = "TestDriveSystem")

    public class TestDriveSystem extends OpMode {

        // lift system code - should be in its own assembly class...

        public static double MAX_POWER = 1;
        public double DRIVE_POWER = 0.5;
        RobotDrive robot;


        @Override
        public void init() {
            robot = new RobotDrive(hardwareMap, telemetry);
            robot.initDriveMotors();
            robot.initImu();
            robot.initSensors();
            robot.resetHeading();
        }

        @Override
        public void loop() {

//            DRIVE_POWER = 0.1;
            //robot.scaleMovement(MAX_POWER, DRIVE_POWER);

            telemetry.addData("heading:", robot.getHeading());
            robot.distanceTelemetry();
            telemetry.addData("ColorSensor: ", robot.bottomColor.getReading());
            telemetry.addData("ColorSensor BlueTape?: ", robot.bottomColor.onBlue());
            telemetry.addData("ColorSensor RedTape?: ", robot.bottomColor.onRed());

//            telemetry.addData("Is Skystone?", colorSensor.isSkystone(colorSensor.getReading()));



                robot.universalJoystick(gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x,1,
                        robot.getHeading());



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

            if(gamepad1.a){
                robot.resetHeading();
            }
//            if(gamepad1.a){ // this is overlapping with the lift test controls below...
//                robot.testDriveSlow();
//            }


            if(gamepad1.left_bumper){
                robot.rotateLeft(DRIVE_POWER);
            } else if(gamepad1.right_bumper){
                robot.rotateRight(DRIVE_POWER);
            }


            if(gamepad2.right_bumper){
                robot.rotateToHeading180Left();
            }

            if(gamepad2.dpad_up){
                robot.accelerateInchesForward(1, 40, 7000);
            }else if(gamepad2.dpad_down){
                robot.accelerateInchesBackward(1, 40, 7000);
            }else if(gamepad2.dpad_left){
                robot.accelerateInchesLeft(1, 40, 7000);
            }else if(gamepad2.dpad_right){
                robot.accelerateInchesRight(1, 40, 7000);
            } else if(gamepad2.a){
                robot.stopMotors();
            }
            //add triggers for speed boosts

//            robot.driveTelemetry();
//            robot.telemetryDriveEncoders();
            telemetry.update();

        }
    }


