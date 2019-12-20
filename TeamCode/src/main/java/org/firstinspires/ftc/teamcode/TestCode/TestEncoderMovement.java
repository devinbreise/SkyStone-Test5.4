package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name = "TestEncoderMovement")
//@Disabled

public class TestEncoderMovement extends LinearOpMode {


    RobotDrive robot;
    double lfspeed = 0.5;
    double rfspeed = 0.5;
    double lbspeed = 0.5;
    double rbspeed = 0.5;
    double speedIncrement = .05;
    double distance = 30;
    double distanceIncrement = 1;
    boolean withEncoders = false;


    @Override
    public void runOpMode() throws InterruptedException {
        teamUtil.init(this);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);

        robot = new RobotDrive(hardwareMap, telemetry);
        robot.initDriveMotors();
        robot.initImu();
        robot.initSensors();

        robot.resetAllDriveEncoders();
        robot.setAllMotorsWithoutEncoder();



        robot.encoderTelemetry();
        robot.driveTelemetry();
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.x) {
                if (gamepad1.dpad_up) {
                    lfspeed = lfspeed + speedIncrement;
                    sleep(500);
                } else if (gamepad1.dpad_down) {
                    lfspeed = lfspeed - speedIncrement;
                    sleep(500);
                }
            } else if (gamepad1.y) {
                if (gamepad1.dpad_up) {
                    rfspeed = rfspeed + speedIncrement;
                    sleep(500);
                } else if (gamepad1.dpad_down) {
                    rfspeed = rfspeed - speedIncrement;
                    sleep(500);
                }
            } else if (gamepad1.a) {
                if (gamepad1.dpad_up) {
                    lbspeed = lbspeed + speedIncrement;
                    sleep(500);
                } else if (gamepad1.dpad_down) {
                    lbspeed = lbspeed - speedIncrement;
                    sleep(500);
                }
            } else if (gamepad1.b) {
                if (gamepad1.dpad_up) {
                    rbspeed = rbspeed + speedIncrement;
                    sleep(500);
                } else if (gamepad1.dpad_down) {
                    rbspeed = rbspeed - speedIncrement;
                    sleep(500);
                }
            } else if (gamepad1.right_bumper) {
                if (gamepad1.dpad_up) {
                    rbspeed = rbspeed + speedIncrement;
                    lfspeed = lfspeed + speedIncrement;
                    lbspeed = lbspeed + speedIncrement;
                    rbspeed = rbspeed + speedIncrement;
                    sleep(500);
                } else if (gamepad1.dpad_down) {
                    rbspeed = rbspeed - speedIncrement;
                    lfspeed = lfspeed - speedIncrement;
                    lbspeed = lbspeed - speedIncrement;
                    rbspeed = rbspeed - speedIncrement;
                    sleep(500);
                }
            } else if (gamepad1.left_bumper) {
                if (gamepad1.dpad_up) {
                    distance = distance + distanceIncrement;
                    sleep(500);
                } else if (gamepad1.dpad_down) {
                    distance = distance - distanceIncrement;
                    sleep(500);
                }
            } else if (gamepad1.left_trigger> .5) {
                robot.setAllMotorsWithoutEncoder();
                withEncoders = false;
                sleep(500);
            } else if (gamepad1.right_trigger> .5) {
                robot.setAllMotorsWithEncoder();
                withEncoders = true;
                sleep(500);
            }else if (gamepad1.dpad_up) {
                robot.moveInches(distance, -lfspeed, rfspeed, -lbspeed, rbspeed, 9000);
            } else if (gamepad1.dpad_down) {
                robot.moveInches(distance, lfspeed, -rfspeed, lbspeed, -rbspeed, 9000);
            } else if (gamepad1.dpad_left) {
                robot.moveInches(distance, lfspeed, rfspeed, -lbspeed, -rbspeed, 9000);
            } else if (gamepad1.dpad_right) {
                robot.moveInches(distance, -lfspeed, -rfspeed, lbspeed, rbspeed, 9000);
            }
            telemetry.addLine("Distance: "+ distance);
            telemetry.addLine("Power LF: "+ lfspeed+" RF: "+ rfspeed+" LB: "+ lbspeed+" RB: "+ rbspeed);
            telemetry.addLine("Encoders:" + withEncoders);
            robot.distanceTelemetry();
            robot.encoderTelemetry();
            robot.driveTelemetry();
            telemetry.update();
        }


    teamUtil.log("BACKLEFT " + robot.getBackLeftMotorPos());
    teamUtil.log("BACKRIGHT " + robot.getBackRightMotorPos());
    teamUtil.log("FRONTLEFT " + robot.getFrontLeftMotorPos());
    teamUtil.log("FRONTRIGHT " + robot.getFrontRightMotorPos());

        }




    }

