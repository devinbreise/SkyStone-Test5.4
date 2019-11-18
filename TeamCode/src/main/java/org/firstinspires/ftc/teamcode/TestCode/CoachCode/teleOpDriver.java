package org.firstinspires.ftc.teamcode.TestCode.CoachCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.basicLibs.revHubIMUGyro;
import org.firstinspires.ftc.teamcode.basicLibs.teamColorSensor;
//import org.firstinspires.ftc.teamcode.basicLibs.xRail;


@TeleOp(name="Coach Test", group="Linear Opmode")
@Disabled

public class teleOpDriver extends LinearOpMode {

    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;
    private DcMotor laMotor;
    //private coachGyro gyro;
    BNO055IMU imu;
    private revHubIMUGyro gyro;
    //private xRail xrail;
    private autoDriver driver;

    private DigitalChannel digitalTouch;
    private teamColorSensor leftColor;
    private teamColorSensor rightColor;
    //private DistanceSensor sensorDistance;
    private DistanceSensor left2m;
    private DistanceSensor right2m;


    //private goBuildAServo2000 servoTest = new goBuildAServo2000();

    private Servo reachServoFTC;
    private Servo grabberServoFTC;
    //private goBuildAServo2000 reachServo;
    //private goBuildAServo2000 grabberServo;
    private coachDetect detector;

    @Override
    public void runOpMode() {

        // set up our IMU
        gyro = new revHubIMUGyro(hardwareMap, telemetry);
        //xrail = new xRail(telemetry, hardwareMap.get(DcMotor.class, "xRailMotor"));

        // Get the objects for the various pieces of hardware
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        leftRearMotor = hardwareMap.get(DcMotor.class, "motorLeft");
        rightRearMotor = hardwareMap.get(DcMotor.class, "motorRight");
        laMotor = hardwareMap.get(DcMotor.class, "LAMotor");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        leftColor = new teamColorSensor(telemetry, hardwareMap.get(ColorSensor.class, "leftRearColor"));
        rightColor = new teamColorSensor(telemetry, hardwareMap.get(ColorSensor.class, "rightRearColor"));
        //sensorDistance = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //grabberServoFTC = hardwareMap.get(Servo.class, "grabberServo");
        //reachServoFTC = hardwareMap.get(Servo.class, "reachServo");
        //vu = new coachVu(telemetry, hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));

        // you can use this as a regular DistanceSensor.
        left2m = hardwareMap.get(DistanceSensor.class, "leftFront2M");
        right2m = hardwareMap.get(DistanceSensor.class, "rightFront2M");
        // you can also cast this to a Rev2mDistanceSensor if you want to use added
        // methods associated with the Rev2mDistanceSensor class.
        //Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange2m;

        //grabberServoFTC.setPosition(0.004 * 0);
        //reachServoFTC.setPosition(0.004 * 0);


        // set digital channel to input mode.
        //digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // reverse the drive on the left rear motor for normal control
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driver = new autoDriver (hardwareMap, this, telemetry, leftRearMotor, rightRearMotor, gyro, leftColor, rightColor, left2m, right2m);

        //xrail.init();

        //vu.init();
        //detector = new coachDetect(telemetry, hardwareMap, hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
        //detector.init();

        // Output status to the console
        telemetry.addData("Status", "Initialized");
        //telemetry.addData("IMU status", imu.getSystemStatus());
        //telemetry.addData("IMU Calib", imu.getCalibrationStatus());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //vu.activate(); // start vuforia tracking
        //detector.start();
        rightColor.calibrate();

        // run until the end of the match (driver presses STOP)
        double tgtPowerLR = 0;
        double tgtPowerRR = 0;
        gyro.resetHeading();
        double reachPos = 0;
        float extendSpeed = 0;
        float heading = 0;
        while (opModeIsActive()) {


            tgtPowerLR = -this.gamepad1.left_stick_y;
            tgtPowerRR = -this.gamepad1.right_stick_y;
            leftRearMotor.setPower(tgtPowerLR);
            rightRearMotor.setPower(tgtPowerRR);

            if(gamepad1.right_stick_button) {
                driver.setZeroHeading();
                driver.smoothMoveInches(1,56);
                driver.spinLeftTo(45);
                driver.smoothMoveInches(1,67);
                driver.spinLeftTo(90);
                driver.smoothMoveInches(1,45);
                driver.motorsOn(0.3);
                driver.rightWaitForLine();
                driver.moveInches(-.3,-12);
                driver.spinLeftTo(180);
                driver.motorsOn(0.3);
                driver.rightWaitForLine();
                driver.smoothMoveInches(1,45);
                driver.spinLeftTo(225);
                driver.smoothMoveInches(1,65);
                driver.spinLeftTo(270);
                driver.smoothMoveInches(1,60);


            } else if(gamepad1.x) {
                //driver.spinTo(1, -90);
                //driver.spin(1, -90);
                heading = heading + 90;
                if (heading > 360) {heading = heading -360;}
                driver.spinLeftTo(heading);
            } else if ( gamepad1.b) {
                //driver.spinTo(1, 90);
                driver.spin(1, 90);
            } else if ( gamepad1.y) {
                //driver.spin(.15, -90);
                driver.smoothMoveInches(1,60);
            } else if ( gamepad1.a) {
                //driver.spin(.15, 90);
                driver.moveInches(-0.5,-36);
            }

            /*else if (gamepad1.dpad_left) {
                grabberServoFTC.setPosition(0.004 * 0);
            } else if (gamepad1.dpad_up) {
                grabberServoFTC.setPosition(0.004 * 35);
            } else if (gamepad1.dpad_right) {
                grabberServoFTC.setPosition(0.004 * 60);
            } */
              else if (gamepad1.dpad_down) {
                //driver.squareOnBlueLine(-.15);
            } else if (gamepad1.dpad_up) {
                  driver.squareOnWall(.2);
            } else if (gamepad1.left_bumper) {
                laMotor.setPower(1);
                //reachPos = reachPos+5;
                //reachServoFTC.setPosition(0.004 * 150);
                sleep(1000);
            } else if (gamepad1.right_bumper) {
                //reachPos = reachPos-5;
                //reachServoFTC.setPosition(0.004 * 0);
                //sleep(1000);
                laMotor.setPower(-1);
            } else {
                laMotor.setPower(0);
            }

            if (gamepad2.right_bumper) {
                extendSpeed = extendSpeed + (float) .1;
                while (gamepad2.right_bumper) {
                }
            } else if (gamepad2.left_bumper) {
                extendSpeed = extendSpeed - (float) .1;
                while (gamepad2.left_bumper) {
                }
            } else if (gamepad2.left_stick_button) {
                  //xrail.loadLander2(extendSpeed);
            }
            telemetry.addData("extendSpeed", extendSpeed);
            telemetry.addData("reachPos", reachPos);




            // generic DistanceSensor methods.
            //telemetry.addData("range", String.format("%.01f mm", sensorRange2m.getDistance(DistanceUnit.MM)));
            //telemetry.addData("range", String.format("%.01f cm", sensorRange2m.getDistance(DistanceUnit.CM)));
            //telemetry.addData("range", String.format("%.01f m", sensorRange2m.getDistance(DistanceUnit.METER)));
            telemetry.addData("left2m: ", String.format("%.01f in", left2m.getDistance(DistanceUnit.INCH)));
            telemetry.addData("right2m: ", String.format("%.01f in", right2m.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            //telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            //test out vuforia vumark navigation
            // vu.getLocation();
            //detector.track();

            //telemetry.addData("Servo Position", servoTest.getPosition());
            //telemetry.addData("Left Motor Target Power", tgtPowerLR);
            //telemetry.addData("Right Motor Target Power", tgtPowerRR);
            //telemetry.addData("Left Motor Power", leftRearMotor.getPower());
            //telemetry.addData("Right Motor Power", rightRearMotor.getPower());
            telemetry.addData("heading", gyro.getHeading());
            telemetry.addData("Drive Encoders",  "L:%7d R:%7d",
                    leftRearMotor.getCurrentPosition(),
                    rightRearMotor.getCurrentPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
        //detector.stop();

    }

}