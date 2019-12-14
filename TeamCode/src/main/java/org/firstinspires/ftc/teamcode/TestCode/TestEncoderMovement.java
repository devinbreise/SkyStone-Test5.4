package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name = "TestEncoderMovement")
public class TestEncoderMovement extends LinearOpMode {


    RobotDrive robot;


    @Override
    public void runOpMode() throws InterruptedException {
        teamUtil.init(this);

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
            if (gamepad1.dpad_up) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.SIGNAL_1);
                robot.moveInchesForward(0.5, 30, 9000);
            }
            if (gamepad1.dpad_down) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.SIGNAL_1);
                robot.moveInchesBackward(0.5, 30, 9000);
            }
            if (gamepad1.dpad_left) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.SIGNAL_2);
                robot.moveInchesLeft(0.5, 30, 9000);
            }
            if (gamepad1.dpad_right) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.SIGNAL_2);
                robot.moveInchesRight(0.5, 30, 9000);
            }
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

