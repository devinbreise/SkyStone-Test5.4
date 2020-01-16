package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.Robot;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "NEW TestDriveSystem")
//@Disabled
public class newTestDrive extends LinearOpMode{

        public static double MAX_POWER = 1;
        public double MAX_VELOCITY = 2200; // encoder tics per second
        RobotDrive drive;
        double targetDistance = 1;
        boolean leftSensor = true;


        public void initialize() {
            teamUtil.init(this);
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
            drive = new RobotDrive(hardwareMap, telemetry);

//        teamGamePad = new TeamGamepad(this);
            drive.initDriveMotors();
            drive.initImu();
            //drive.initSensors();
            drive.resetHeading();
        }
        @Override
        public void runOpMode() throws InterruptedException {
            initialize();
            waitForStart();
            drive.resetHeading();

            while (opModeIsActive()) {

                drive.universalJoystick(gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x, 1,
                        drive.getHeading());


                if (gamepad1.x) {
                    if (gamepad1.dpad_up) {
                        targetDistance = targetDistance+1;
                        sleep(250);
                    } else if (gamepad1.dpad_down) {
                        targetDistance = targetDistance+1;
                        sleep(250);
                    }
                } else if (gamepad1.y) {
                    drive.newMoveToDistance(leftSensor ? drive.frontLeftDistance : drive.frontRightDistance, targetDistance, 2200, 0, 5000);
                }
                if (gamepad1.dpad_up) {
                    drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_FIELD);
                } else if (gamepad1.dpad_down) {
                    drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_DRIVER);
                } else if (gamepad1.dpad_left) {
                    drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_DEPOT);
                } else if (gamepad1.dpad_right ) {
                    drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_BUILDING);
                } else

                if (gamepad1.left_bumper) {
                    if ( teamUtil.alliance == teamUtil.Alliance.BLUE) {
                        teamUtil.alliance = teamUtil.Alliance.RED;
                    } else {
                        teamUtil.alliance = teamUtil.Alliance.BLUE;
                    }
                    sleep(250);
                }
                if (gamepad1.right_bumper) {
                    if ( leftSensor) {
                        leftSensor = false;
                    } else {
                        leftSensor = true;
                    }
                    sleep(250);
                }


                teamUtil.telemetry.addData("heading:", drive.getHeading());
                teamUtil.telemetry.addData("Alliance:", teamUtil.alliance);
                if (leftSensor) {
                    teamUtil.telemetry.addData("Left Sensor, Target Distance:", targetDistance);
                } else {
                    teamUtil.telemetry.addData("Right Sensor, Target Distance:", targetDistance);
                }

                //drive.distanceTelemetry();
                //drive.telemetryDriveEncoders();
                telemetry.update();
            }
        }
    }


