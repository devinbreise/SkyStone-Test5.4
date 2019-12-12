package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.teamColorSensor;


@TeleOp(name = "TestDriveSystem")
    public class TestDriveSystem extends OpMode {

        // lift system code - should be in its own assembly class...

        public static double MAX_POWER = 1;
        public static double DRIVE_POWER;
        RobotDrive robot;
        teamColorSensor colorSensor;


        @Override
        public void init() {
            robot = new RobotDrive(hardwareMap, telemetry);
            colorSensor = new teamColorSensor(telemetry, hardwareMap.get(ColorSensor.class, "colorSensor"));
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
            telemetry.addData("ColorSensor: ", colorSensor.getReading());
            telemetry.addData("Is Skystone?", colorSensor.isSkystone(colorSensor.getReading()));


            if(gamepad1.left_trigger > 0.1){
                telemetry.addData("", "REGULAR DRIVE");
                robot.driveJoyStick(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x,1);

            } else {
                telemetry.addData("", "IMU DRIVE");

                robot.universalJoystick(gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x,1,
                        robot.getHeading());
            }
//TESIGN IMU CODE
//                if(gamepad1.b){
//                    robot.universalJoystick(0,
//                            -0.5f,
//                            gamepad1.right_stick_x,
//                            45);
//
//                } else robot.stopMotors();
//            }


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
            //add triggers for speed boosts

//            robot.driveTelemetry();
//            robot.telemetryDriveEncoders();
            telemetry.update();

        }
    }


