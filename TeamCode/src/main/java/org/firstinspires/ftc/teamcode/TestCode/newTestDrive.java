package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@TeleOp(name = "NEW TestDriveSystem")
//@Disabled
public class newTestDrive extends LinearOpMode{

        public static double MAX_POWER = 1;
        public double MAX_VELOCITY = 2200; // encoder tics per second
        RobotDrive drive;
    boolean wasTurning = false;
    double storedHeading;
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
            drive.initSensors(false);
            drive.resetHeading();
        }

        public void stressTestDistanceSensors() {
            drive.newMoveToDistance(drive.frontLeftDistance, 5, 2200, 0, false, 5000);
            drive.newMoveToDistance(drive.frontLeftDistance, 10, 2200, 0, true, 5000);
            drive.newRotateTo(270);
            drive.newAccelerateInchesForward(2200,90,268,5000);
            drive.newMoveToDistance(drive.frontLeftDistance, 5, 2200, 270, false, 5000);
            drive.newMoveToDistance(drive.frontLeftDistance, 10, 2200, 270, true, 5000);
            drive.newRotateTo(0);
            drive.newMoveToDistance(drive.frontLeftDistance, 5, 2200, 0, false, 5000);
            drive.newMoveToDistance(drive.frontLeftDistance, 10, 2200, 0, true, 5000);
            drive.newRotateTo(90);
            drive.newAccelerateInchesForward(2200,100,88,5000);
            drive.newMoveToDistance(drive.frontLeftDistance, 5, 2200, 90, false, 5000);
            drive.newMoveToDistance(drive.frontLeftDistance, 10, 2200, 90, true, 5000);
            drive.newRotateTo(0);
        }

        @Override
        public void runOpMode() throws InterruptedException {
            initialize();
            waitForStart();
            drive.resetHeading();

            while (opModeIsActive()) {

                if(Math.abs(gamepad1.right_stick_x) < 0.1 && wasTurning){
                    storedHeading = drive.getHeading();
                }
                drive.universalJoystick(gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x, 1,
                        drive.getHeading(), storedHeading);


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
                    if (gamepad1.right_bumper) {
                        drive.newAccelerateInchesForward(-2200, travelDistance, drive.getHeading() - 1.5, 5000);
                    } else {
                        drive.newAccelerateInchesForward(2200, travelDistance, drive.getHeading() - 2, 5000);
                        //drive.newAccelerateInchesForward(2200, travelDistance, 0, 5000);
                    }
                }else if (gamepad1.x) {
                    if (gamepad1.dpad_up) {
                        targetDistance = targetDistance+1;
                        sleep(250);
                    } else if (gamepad1.dpad_down) {
                        targetDistance = targetDistance-1;
                        sleep(250);
                    }
                } else if (gamepad1.y) {
                    drive.newMoveToDistance(leftSensor ? drive.frontLeftDistance : drive.frontRightDistance, targetDistance, 500, 0, true, 60000);
                } else
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
                } else
                if (gamepad1.right_bumper) {
                    leftSensor = !leftSensor;
                    sleep(250);
                } else
                if (gamepad1.left_trigger > 0.8) {
                    demoAutoDrive();
                } else
                if (gamepad1.right_trigger > 0.8) {
                    demoAutoDriveNoSensors();
                    //drive.newPositionToFoundation(0, 6000);
                }

                if (gamepad2.right_bumper) {
                    long start = System.currentTimeMillis();
                    long readings = 0;
                    while (gamepad2.right_bumper) {
                        teamUtil.log("D:" + drive.frontLeftDistance.getDistance());
                        readings+=1;
                    }
                    long seconds = (System.currentTimeMillis()-start)/1000;
                    teamUtil.log("Num Readings:" + readings + " Seconds:" + seconds +" Readings/Sec:" + (readings/seconds));
                }
                if (gamepad2.left_bumper) {
                    long start = System.currentTimeMillis();
                    long readings = 0;
                    while (gamepad2.left_bumper) {
                        readings+=1;
                        teamUtil.log("D:" + drive.frontLeftDistance.getDistance()+":"
                                +drive.frontRightDistance.getDistance()+":"
                                +drive.leftDistanceSensor.getDistance() +":"
                                +drive.rightDistanceSensor.getDistance()+":"
                                +drive.backDistanceSensor.getDistance()+":");
                    }
                    long seconds = (System.currentTimeMillis()-start)/1000;
                    teamUtil.log("Num Readings:" + readings + " Seconds:" + seconds +" Readings/Sec:" + (readings/seconds));
                }
                if (gamepad2.x) {
                    stressTestDistanceSensors();
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
            drive.newAccelerateInchesForward(1500,25,0,3000);
            drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_FIELD);
            teamUtil.theBlinkin.flash(RevBlinkinLedDriver.BlinkinPattern.GREEN, 500);
            drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_BUILDING);
            drive.newAccelerateInchesForward(2200,62,teamUtil.alliance==teamUtil.Alliance.RED ? 268: 88,5000);
            drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_FIELD);
            drive.newPositionToFoundation(0, 4000);
            teamUtil.theBlinkin.flash(RevBlinkinLedDriver.BlinkinPattern.GREEN, 500);
            drive.newAccelerateInchesForward(-2200,3,0,3000);
            drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_DEPOT);
            drive.newAccelerateInchesForward(2200,83,teamUtil.alliance==teamUtil.Alliance.RED ? 88: 268,5000);
            drive.newMoveToDistance(drive.frontRightDistance, 10.5,1500,teamUtil.alliance==teamUtil.Alliance.RED ? 89: 271 , true,4000);
            drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_FIELD);
            drive.newMoveToDistance(drive.frontLeftDistance, 6,1500,0 , true,4000);
            teamUtil.theBlinkin.flash(RevBlinkinLedDriver.BlinkinPattern.GREEN, 500);
            drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_BUILDING);
            drive.newAccelerateInchesForward(2200,70,teamUtil.alliance==teamUtil.Alliance.RED ? 269: 89,5000);
            teamUtil.theBlinkin.flash(RevBlinkinLedDriver.BlinkinPattern.GREEN, 500);
            drive.newAccelerateInchesForward(-2200,10,teamUtil.alliance==teamUtil.Alliance.RED ? 271: 91,2000);
        }

    void demoAutoDriveNoSensors () {
        drive.newAccelerateInchesForward(1600,25,0,3000);
        drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_FIELD);
        teamUtil.theBlinkin.flash(RevBlinkinLedDriver.BlinkinPattern.GREEN, 500);
        drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_BUILDING);
        drive.newAccelerateInchesForward(2200,64.5,teamUtil.alliance==teamUtil.Alliance.RED ? 268: 88,5000);
        drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_FIELD);
        drive.newAccelerateInchesForward(2200,3.5,0,3000);
        teamUtil.theBlinkin.flash(RevBlinkinLedDriver.BlinkinPattern.GREEN, 500);
        drive.newAccelerateInchesForward(-2200,5,0,3000);
        drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_DEPOT);
        drive.newAccelerateInchesForward(2200,89.5,teamUtil.alliance==teamUtil.Alliance.RED ? 88: 268,5000);
        drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_FIELD);
        drive.newAccelerateInchesForward(2200,6,0,3000);
        teamUtil.theBlinkin.flash(RevBlinkinLedDriver.BlinkinPattern.GREEN, 500);
        teamUtil.theBlinkin.flash(RevBlinkinLedDriver.BlinkinPattern.GREEN, 500);
        drive.newAccelerateInchesForward(-2200,3,0,3000);
        drive.newRotateTo(RobotDrive.RobotRotation.TOWARDS_BUILDING);
        drive.newAccelerateInchesForward(2200,70,teamUtil.alliance==teamUtil.Alliance.RED ? 269: 89,5000);
        teamUtil.theBlinkin.flash(RevBlinkinLedDriver.BlinkinPattern.GREEN, 500);
        drive.newAccelerateInchesForward(-2200,7,teamUtil.alliance==teamUtil.Alliance.RED ? 271: 91,2000);
    }
}


