package org.firstinspires.ftc.teamcode.TestCode.CoachCode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Assemblies.RobotDrive;
import org.firstinspires.ftc.teamcode.basicLibs.Blinkin;
import org.firstinspires.ftc.teamcode.basicLibs.DistanceSensors;
import org.firstinspires.ftc.teamcode.basicLibs.revHubIMUGyro;
import org.firstinspires.ftc.teamcode.basicLibs.teamColorSensor;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

@Autonomous(name = "mechnumDriveTest")
@Disabled
public class mechnumDriveTest extends LinearOpMode {

    // Stuff that belongs in a drive class
    private double COUNTS_PER_INCH = 62.24;
    private double COUNTS_PER_INCH_SIDEWAYS = 67.82;

    public static double INITIAL_HEADING;

    //HardwareMap hardwareMap;
    //Telemetry telemetry;
    boolean timedOut = false;
    DcMotor fLeftMotor;
    DcMotor bLeftMotor;
    DcMotor fRightMotor;
    DcMotor bRightMotor;
    public DistanceSensors frontLeftDistance;
    public DistanceSensors frontRightDistance;
    public DistanceSensors leftDistanceSensor;
    public DistanceSensors rightDistanceSensor;
    public DistanceSensors backDistanceSensor;
    public DistanceSensor frontmiddleDistance;
    public ColorSensor frontmiddleColor;
    public ColorSensor bottomColorSensor;
    public teamColorSensor bottomColor;

    // stuff that is just for testing
    double lfspeed = 0.1;
    double rfspeed = 0.1;
    double lbspeed = 0.1;
    double rbspeed = 0.1;
    double speedIncrement = .025;
    int lfEncoder, rfEncoder, lbEncoder, rbEncoder;


    //    Servo latchOne;
//    Servo latchTwo;
    revHubIMUGyro revImu;

    public void initImu() {
        revImu = new revHubIMUGyro(hardwareMap, telemetry);

    }

    public void resetHeading() {
        INITIAL_HEADING = revImu.getHeading();
    }

    public double getAbsoluteHeading() {
        return revImu.getAbsoluteHeading();
    }

    public double getHeading() {
        return revImu.correctHeading(revImu.getHeading() - INITIAL_HEADING);
    }

    //Your class (and all classes representing functional parts of the robot) should have a constructor
    // to set themselves up as well as an "initialize" method that would be called during the robot initialization and
    // possibly a "start" method that would be called just after start and maybe some "shutdown" methods as well.
    // In other words, they should look a bit like a standard OpMode class.
    // We should figure out what our "standard" approach is going to be for acquiring things from the FTC hardwareMap and initialization.
    // For example, should each class representing a particular assembly on the robot (like the drive)
    // reach into the hardwareMap and get the needed objects (and thus encapsulate the names of those things
    // in the configuration file) or should the overall "robot" class do all that work and pass the needed
    // objects down into the assembly level classes?  Or maybe a hybrid approach where all the "names" in the
    // config file are in one place but the assembly classes do the initialization work...Getting a bit ahead here!

    public void initDriveMotors() {
        fLeftMotor = hardwareMap.dcMotor.get("fLeftMotor");
        fRightMotor = hardwareMap.dcMotor.get("fRightMotor");
        bLeftMotor = hardwareMap.dcMotor.get("bLeftMotor");
        bRightMotor = hardwareMap.dcMotor.get("bRightMotor");
        fLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        setAllMotorsWithEncoder();
        setBrakeAllDriveMotors();
    }


    public void initSensors() {
        frontLeftDistance = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "frontLeftDistance"));
        frontRightDistance = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "frontRightDistance"));
        leftDistanceSensor = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "leftDistance"));
        rightDistanceSensor = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "rightDistance"));
        backDistanceSensor = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "backDistance"));
        frontmiddleDistance = hardwareMap.get(DistanceSensor.class, "frontColorSensor");
        frontmiddleColor = hardwareMap.get(ColorSensor.class, "frontColorSensor");
        bottomColorSensor = hardwareMap.get(ColorSensor.class, "bottomColorSensor");
        bottomColor = new teamColorSensor(telemetry, bottomColorSensor);
        bottomColor.calibrate();
    }

    public void setAllMotorsWithoutEncoder() {
        fLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setAllMotorsWithEncoder() {
        fLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void runAllMotorsToPosition() {
        fLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setBrakeAllDriveMotors() {
        fLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetAllDriveEncoders() {
        fLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stopMotors() {
        fRightMotor.setPower(0);
        fLeftMotor.setPower(0);
        bRightMotor.setPower(0);
        bLeftMotor.setPower(0);
    }

    public void onForward() {
        teamUtil.log("Encoders LF:" + fLeftMotor.getCurrentPosition() + " RF:" + fRightMotor.getCurrentPosition() + " LB:" + bLeftMotor.getCurrentPosition() + " RB:" + bRightMotor.getCurrentPosition());
        fRightMotor.setPower(rfspeed);
        fLeftMotor.setPower(lfspeed);
        bRightMotor.setPower(rbspeed);
        bLeftMotor.setPower(lbspeed);
        while (gamepad1.dpad_up){
            teamUtil.log("Encoders LF:" + fLeftMotor.getCurrentPosition() + " RF:" + fRightMotor.getCurrentPosition() + " LB:" + bLeftMotor.getCurrentPosition() + " RB:" + bRightMotor.getCurrentPosition());
        }
    }
    public void onBackward() {
        teamUtil.log("Encoders LF:" + fLeftMotor.getCurrentPosition() + " RF:" + fRightMotor.getCurrentPosition() + " LB:" + bLeftMotor.getCurrentPosition() + " RB:" + bRightMotor.getCurrentPosition());
        fRightMotor.setPower(-rfspeed);
        fLeftMotor.setPower(-lfspeed);
        bRightMotor.setPower(-rbspeed);
        bLeftMotor.setPower(-lbspeed);
        while (gamepad1.dpad_down){
            teamUtil.log("Encoders LF:" + fLeftMotor.getCurrentPosition() + " RF:" + fRightMotor.getCurrentPosition() + " LB:" + bLeftMotor.getCurrentPosition() + " RB:" + bRightMotor.getCurrentPosition());
        }
    }
    public void onLeft() {
        teamUtil.log("Encoders LF:" + fLeftMotor.getCurrentPosition() + " RF:" + fRightMotor.getCurrentPosition() + " LB:" + bLeftMotor.getCurrentPosition() + " RB:" + bRightMotor.getCurrentPosition());
        fRightMotor.setPower(rfspeed);
        fLeftMotor.setPower(-lfspeed);
        bRightMotor.setPower(-rbspeed);
        bLeftMotor.setPower(lbspeed);
        while (gamepad1.dpad_left) {
            teamUtil.log("Encoders LF:" + fLeftMotor.getCurrentPosition() + " RF:" + fRightMotor.getCurrentPosition() + " LB:" + bLeftMotor.getCurrentPosition() + " RB:" + bRightMotor.getCurrentPosition());
        }
    }
    public void onRight() {
        teamUtil.log("Encoders LF:" + fLeftMotor.getCurrentPosition() + " RF:" + fRightMotor.getCurrentPosition() + " LB:" + bLeftMotor.getCurrentPosition() + " RB:" + bRightMotor.getCurrentPosition());
        fRightMotor.setPower(-rfspeed);
        fLeftMotor.setPower(lfspeed);
        bRightMotor.setPower(rbspeed);
        bLeftMotor.setPower(-lbspeed);
        while (gamepad1.dpad_right) {
            teamUtil.log("Encoders LF:" + fLeftMotor.getCurrentPosition() + " RF:" + fRightMotor.getCurrentPosition() + " LB:" + bLeftMotor.getCurrentPosition() + " RB:" + bRightMotor.getCurrentPosition());
        }
    }

    // Try to match the encoder counts on all 4 wheels to the lf motor by
    // varying the power to the other 3 proportionally
    public void pForward(double basePower) {
        double pfactor = .001;
        double lfPower = basePower;
        double rfPower = basePower;
        double lbPower = basePower;
        double rbPower = basePower;
        resetAllDriveEncoders();
        setAllMotorsWithEncoder();
        do {
            fRightMotor.setPower(rfPower);
            fLeftMotor.setPower(lfPower);
            bRightMotor.setPower(rbPower);
            bLeftMotor.setPower(lbPower);

            lfEncoder = fLeftMotor.getCurrentPosition();
            rfEncoder = fRightMotor.getCurrentPosition();
            lbEncoder = bLeftMotor.getCurrentPosition();
            rbEncoder = bRightMotor.getCurrentPosition();
            rfPower =  basePower - (rfEncoder - lfEncoder) * pfactor;
            lbPower = basePower - (lbEncoder - lfEncoder) * pfactor;
            rbPower =  basePower - (rbEncoder - lfEncoder) * pfactor;

            teamUtil.log("Encoders LF:"+ lfEncoder+" RF:" + rfEncoder+" LB:" + lbEncoder+" RB:"+ rbEncoder);

        } while (gamepad1.dpad_up);

        stopMotors();
    }

    public void closeToDistance (double distance, double initialPower) {
        double minPower = .1;
        double decelerateInches = 2;
        double power = initialPower;
        double currentDistance  = frontLeftDistance.getDistance();
        double slope = (initialPower-minPower) / (currentDistance-distance);
        if (currentDistance > distance) {
            fRightMotor.setPower(power);
            fLeftMotor.setPower(power);
            bRightMotor.setPower(power);
            bLeftMotor.setPower(power);
            while (currentDistance > distance + decelerateInches) {
                teamUtil.log("Distance:" + currentDistance + " Power:" + power);
                currentDistance = frontLeftDistance.getDistance();
            }
            while (currentDistance > distance) {
                teamUtil.log("Distance:" + currentDistance + " Power:" + power);
                power = (currentDistance - distance) * slope + minPower;
                fRightMotor.setPower(power);
                fLeftMotor.setPower(power);
                bRightMotor.setPower(power);
                bLeftMotor.setPower(power);
                currentDistance = frontLeftDistance.getDistance();
            }
        }
        stopMotors();
    }

    public void distanceTelemetry() {
        telemetry.addData("frontLeftDistance", frontLeftDistance.getDistance());
        telemetry.addData("frontMiddleDistance", frontmiddleDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("frontRightDistance", frontRightDistance.getDistance());
        telemetry.addData("leftDistance", leftDistanceSensor.getDistance());
        telemetry.addData("rightDistance", rightDistanceSensor.getDistance());
        telemetry.addData("backDistance", backDistanceSensor.getDistance());
        telemetry.addLine("front color"+frontmiddleColor.alpha()+":" +frontmiddleColor.red()+":" +frontmiddleColor.green()+":" +frontmiddleColor.blue());
    }

    public void telemetryDriveEncoders() {
        telemetry.addData("front left:", fLeftMotor.getCurrentPosition());
        telemetry.addData("front right:", fRightMotor.getCurrentPosition());
        telemetry.addData("back left:", bLeftMotor.getCurrentPosition());
        telemetry.addData("back right:", bRightMotor.getCurrentPosition());
    }

    public void driveTelemetry() {
        telemetry.addData("Front Left Motor:", fLeftMotor.getPower());
        telemetry.addData("Front Right Motor:", fRightMotor.getPower());
        telemetry.addData("Back Left Motor:", bLeftMotor.getPower());
        telemetry.addData("Back Right Motor:", bRightMotor.getPower());
        telemetry.addData("Absolute Heading:", getAbsoluteHeading());

    }

    public void logEncoders () {
        lfEncoder = fLeftMotor.getCurrentPosition();
        rfEncoder = fRightMotor.getCurrentPosition();
        lbEncoder = bLeftMotor.getCurrentPosition();
        rbEncoder = bRightMotor.getCurrentPosition();
        teamUtil.log("Encoders LF:"+ lfEncoder+" RF:" + rfEncoder+" LB:" + lbEncoder+" RB:"+ rbEncoder);
    }

    public void logEncoderDistances () {
        teamUtil.log("Encoder Distances LF:"+ (fLeftMotor.getCurrentPosition()-lfEncoder)
                +" RF:" + (fRightMotor.getCurrentPosition()-rfEncoder)
                +" LB:" + (bLeftMotor.getCurrentPosition()-lbEncoder)
                +" RB:"+ (bRightMotor.getCurrentPosition()-rbEncoder));
    }

    public void runOpMode() throws InterruptedException {
        teamUtil.init(this);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);

        initDriveMotors();
        initImu();
        initSensors();

        telemetryDriveEncoders();
        distanceTelemetry();
        telemetry.addData("Heading:", getAbsoluteHeading());
        telemetry.addData("Waiting to Start:", 0);
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.x) {
                if (gamepad1.dpad_up) {
                    lfspeed = lfspeed + speedIncrement;
                    sleep(250);
                } else if (gamepad1.dpad_down) {
                    lfspeed = lfspeed - speedIncrement;
                    sleep(250);
                }
            } else if (gamepad1.y) {
                if (gamepad1.dpad_up) {
                    rfspeed = rfspeed + speedIncrement;
                    sleep(250);
                } else if (gamepad1.dpad_down) {
                    rfspeed = rfspeed - speedIncrement;
                    sleep(250);
                }
            } else if (gamepad1.a) {
                if (gamepad1.dpad_up) {
                    lbspeed = lbspeed + speedIncrement;
                    sleep(250);
                } else if (gamepad1.dpad_down) {
                    lbspeed = lbspeed - speedIncrement;
                    sleep(250);
                }
            } else if (gamepad1.b) {
                if (gamepad1.dpad_up) {
                    rbspeed = rbspeed + speedIncrement;
                    sleep(250);
                } else if (gamepad1.dpad_down) {
                    rbspeed = rbspeed - speedIncrement;
                    sleep(250);
                }
            } else if (gamepad1.right_bumper) {
                if (gamepad1.dpad_up) {
                    rbspeed = rbspeed + speedIncrement;
                    lbspeed = lbspeed + speedIncrement;
                    rfspeed = rfspeed + speedIncrement;
                    lfspeed = lfspeed + speedIncrement;
                    sleep(250);
                } else if (gamepad1.dpad_down) {
                    rbspeed = rbspeed - speedIncrement;
                    lbspeed = lbspeed - speedIncrement;
                    rfspeed = rfspeed - speedIncrement;
                    lfspeed = lfspeed - speedIncrement;
                    sleep(250);
                }
            } else {
                if (gamepad1.dpad_up) {
                    if (gamepad1.left_bumper) {
                        closeToDistance(1,.3);
                        //pForward(0.3);
                    } else {
                        onForward();
                    }
                } else if (gamepad1.dpad_down) {
                    onBackward();
                } else if (gamepad1.dpad_left) {
                    onLeft();
                } else if (gamepad1.dpad_right) {
                    onRight();
                } else {
                    stopMotors();
                }
                if (gamepad1.left_trigger > .5) {
                    logEncoders();
                    sleep(250);
                } else if (gamepad1.right_trigger > .5) {
                    logEncoderDistances();
                    sleep(250);
                }
                if (gamepad1.left_stick_button) {
                    resetAllDriveEncoders();
                    setAllMotorsWithEncoder();
                }
            }

            telemetry.addLine("Power LF: "+ lfspeed+" RF: "+ rfspeed+" LB: "+ lbspeed+" RB: "+ rbspeed);

            //telemetryDriveEncoders();
            //distanceTelemetry();
            //telemetry.addData("Heading:", getHeading());
            //telemetry.addData("AbsoluteHeading:", getAbsoluteHeading());
            telemetry.update();

        }
    }
}
