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
        double travelDistance = 12;
        boolean leftSensor = true;


        public void initialize() {
            teamUtil.init(this);
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
            drive = new RobotDrive(hardwareMap, telemetry);

//        teamGamePad = new TeamGamepad(this);
            drive.initDriveMotors();
            drive.setBrakeAllDriveMotors();
            drive.initImu();
            drive.initSensors(true);
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


                if (gamepad1.a) {
                    if (gamepad1.dpad_up) {
                        if (travelDistance > 12) {
                            travelDistance=travelDistance+12;
                        } else {
                            travelDistance=travelDistance+1;
                        }
                        sleep(250);
                    } else if (gamepad1.dpad_down) {
                        if (travelDistance > 12) {
                            travelDistance=travelDistance-12;
                        } else {
                            travelDistance=travelDistance-1;
                        }
                        sleep(250);
                    }
                } else if (gamepad1.b) {
                    drive.newAccelerateInchesForward( 2200, travelDistance, drive.getHeading()-2, 5000);
                }else if (gamepad1.x) {
                    if (gamepad1.dpad_up) {
                        targetDistance = targetDistance+1;
                        sleep(250);
                    } else if (gamepad1.dpad_down) {
                        targetDistance = targetDistance+1;
                        sleep(250);
                    }
                } else if (gamepad1.y) {
                    drive.newMoveToDistance(leftSensor ? drive.frontLeftDistance : drive.frontRightDistance, targetDistance, 1500, 0, true, 5000);
                }
                if (gamepad1.dpad_up) {
                    drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_FIELD);
                    sleep(250);
                } else if (gamepad1.dpad_down) {
                    drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_DRIVER);
                    sleep(250);
                } else if (gamepad1.dpad_left) {
                    drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_DEPOT);
                    sleep(250);
                } else if (gamepad1.dpad_right ) {
                    drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_BUILDING);
                    sleep(250);
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
                    leftSensor = !leftSensor;
                    sleep(250);
                }
                if (gamepad1.left_trigger > 0.8) {
                    demoAutoDrive();
                }
                if (gamepad1.right_trigger > 0.8) {
                    drive.newPositionToFoundation(0, 6000);
                }

                teamUtil.telemetry.addData("heading:", drive.getHeading());
                teamUtil.telemetry.addData("Absolute heading:", drive.getAbsoluteHeading());

                teamUtil.telemetry.addData("Alliance:", teamUtil.alliance);
                if (leftSensor) {
                    teamUtil.telemetry.addData("Left Sensor, Target Distance:", targetDistance);
                } else {
                    teamUtil.telemetry.addData("Right Sensor, Target Distance:", targetDistance);
                }
                teamUtil.telemetry.addData("Travel Distance:", travelDistance);

                drive.distanceTelemetry();
                //drive.telemetryDriveEncoders();
                telemetry.update();
            }
        }

        // assumes starts in usual spot for skystone auto
        void demoAutoDrive () {
            drive.newAccelerateInchesForward(1800,25,0,3000);
            drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_FIELD);
            sleep(1000);
            drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_BUILDING);
            drive.newAccelerateInchesForward(2200,62,teamUtil.alliance==teamUtil.Alliance.RED ? 268: 92,5000);
            drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_FIELD);
            drive.newPositionToFoundation(0, 4000);
            sleep(1000);
            drive.moveInchesBackward(0.30, 3, 2000);
            drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_DEPOT);
            drive.newAccelerateInchesForward(2200,83,teamUtil.alliance==teamUtil.Alliance.RED ? 88: 272,5000);
            drive.newMoveToDistance(drive.frontRightDistance, 10.5,1500,teamUtil.alliance==teamUtil.Alliance.RED ? 89: 271 , true,4000);
            drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_FIELD);
            drive.newMoveToDistance(drive.frontLeftDistance, 6,1500,0 , true,4000);
            sleep(1000);
            drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_BUILDING);
            drive.newAccelerateInchesForward(2200,70,teamUtil.alliance==teamUtil.Alliance.RED ? 269: 91,5000);
            sleep(1000);
            drive.moveInchesBackward(0.5, 10, 2000);
        }
    }


