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

    //left powers:
    //fl: 0.65
    //fr: 0.65 => 0.625
    //bl: 0.55
    //br: 0.60  => 0.575
    RobotDrive robot;
    double lfspeed = 0.65;
    double rfspeed = 0.65;
    double lbspeed = 0.65;
    double rbspeed = 0.65;
    double testSpeed = 0.5;
    double testInches = 0.5;
    double speedIncrement = .025;
    double distance = 48;
    double distanceIncrement = 1;
    boolean withEncoders = false;

    double lfSpeedPercentage = 1;
    double rfSpeedPercentage = 1;
    double lbSpeedPercentage = 1;
    double rbSpeedPercentage = 1;


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
            } else if (gamepad1.left_trigger > .5) {
                robot.setAllMotorsWithoutEncoder();
                withEncoders = false;
                sleep(500);
            } else if (gamepad1.right_trigger > .5) {
                robot.setAllMotorsWithEncoder();
                withEncoders = true;
                sleep(500);
            } else if (gamepad1.dpad_up) {
                robot.moveInches(distance, lfspeed, rfspeed, lbspeed, rbspeed, 9000);
            } else if (gamepad1.dpad_down) {
                robot.moveInches(distance, -lfspeed, -rfspeed, -lbspeed, -rbspeed, 9000);
            } else if (gamepad1.dpad_left) {
                robot.moveInches(distance, -lfspeed, rfspeed, lbspeed, -rbspeed, 9000);
            } else if (gamepad1.dpad_right) {
                robot.moveInches(distance, lfspeed, -rfspeed, -lbspeed, rbspeed, 9000);
            }

///////////////////////////////////////////////////////////////////////////////////////////////////
//GAMEPAD2


            if (gamepad2.x) {
                if (gamepad2.dpad_up) {
                    lfSpeedPercentage = lfSpeedPercentage + speedIncrement;
                    sleep(500);
                } else if (gamepad2.dpad_down) {
                    lfSpeedPercentage = lfSpeedPercentage - speedIncrement;
                    sleep(500);
                }
            } else if (gamepad2.y) {
                if (gamepad2.dpad_up) {
                    rfSpeedPercentage = rfSpeedPercentage + speedIncrement;
                    sleep(500);
                } else if (gamepad2.dpad_down) {
                    rfSpeedPercentage = rfSpeedPercentage - speedIncrement;
                    sleep(500);
                }
            } else if (gamepad2.a) {
                if (gamepad2.dpad_up) {
                    lbSpeedPercentage = lbSpeedPercentage + speedIncrement;
                    sleep(500);
                } else if (gamepad2.dpad_down) {
                    lbSpeedPercentage = lbSpeedPercentage - speedIncrement;
                    sleep(500);
                }
            } else if (gamepad2.b) {
                if (gamepad2.dpad_up) {
                    rbSpeedPercentage = rbSpeedPercentage + speedIncrement;
                    sleep(500);
                } else if (gamepad2.dpad_down) {
                    rbSpeedPercentage = rbSpeedPercentage - speedIncrement;
                    sleep(500);
                }
            } else if (gamepad2.right_bumper) {
                if (gamepad2.dpad_up) {
                    lfSpeedPercentage = lfSpeedPercentage + speedIncrement;
                    rfSpeedPercentage = rfSpeedPercentage + speedIncrement;
                    lbSpeedPercentage = lbSpeedPercentage + speedIncrement;
                    rbSpeedPercentage = rbSpeedPercentage + speedIncrement;
                    sleep(500);
                } else if (gamepad2.dpad_down) {
                    lfSpeedPercentage = lfSpeedPercentage - speedIncrement;
                    rfSpeedPercentage = rfSpeedPercentage - speedIncrement;
                    lbSpeedPercentage = lbSpeedPercentage - speedIncrement;
                    rbSpeedPercentage = rbSpeedPercentage - speedIncrement;
                    sleep(500);
                }
            } else if (gamepad2.left_bumper) {
                if (gamepad2.dpad_up) {
                    distance = distance + distanceIncrement;
                    sleep(500);
                } else if (gamepad2.dpad_down) {
                    distance = distance - distanceIncrement;
                    sleep(500);
                }


                if (gamepad2.dpad_up) {
                    robot.moveInches(distance, lfspeed * lfSpeedPercentage, rfspeed * rfSpeedPercentage, lbspeed * lbSpeedPercentage, rbspeed * rbSpeedPercentage, 9000);
                } else if (gamepad2.dpad_down) {
                    robot.moveInches(distance, -lfspeed * lfSpeedPercentage, -rfspeed * rfSpeedPercentage, -lbspeed * lbSpeedPercentage, -rbspeed * rbSpeedPercentage, 9000);
                } else if (gamepad2.dpad_left) {
                    robot.moveInches(distance, -lfspeed * lfSpeedPercentage, rfspeed * rfSpeedPercentage, lbspeed * lbSpeedPercentage, -rbspeed * rbSpeedPercentage, 9000);
                } else if (gamepad2.dpad_right) {
                    robot.moveInches(distance, lfspeed * lfSpeedPercentage, -rfspeed * rfSpeedPercentage, -lbspeed * lbSpeedPercentage, rbspeed * rbSpeedPercentage, 9000);
                }

            }

            telemetry.addLine("Distance: " + distance);
            telemetry.addLine("TestSpeed: " + testSpeed);
            telemetry.addLine("Power LF: "+ lfspeed+" RF: "+ rfspeed+" LB: "+ lbspeed+" RB: "+ rbspeed);
            telemetry.addLine("LF%:" + lfSpeedPercentage + " RF%: " + rfSpeedPercentage + " LB%: " + lbSpeedPercentage + " RB%: " + rbSpeedPercentage);
            telemetry.addLine("Encoders:" + withEncoders);
            robot.distanceTelemetry();
            robot.encoderTelemetry();
            robot.driveTelemetry();
            telemetry.update();


            teamUtil.log("BACKLEFT " + robot.getBackLeftMotorPos());
            teamUtil.log("BACKRIGHT " + robot.getBackRightMotorPos());
            teamUtil.log("FRONTLEFT " + robot.getFrontLeftMotorPos());
            teamUtil.log("FRONTRIGHT " + robot.getFrontRightMotorPos());

        }


    }
}
