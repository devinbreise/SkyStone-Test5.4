package org.firstinspires.ftc.teamcode.Assemblies;


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.basicLibs.DistanceSensors;
import org.firstinspires.ftc.teamcode.basicLibs.revHubIMUGyro;
import org.firstinspires.ftc.teamcode.basicLibs.teamColorSensor;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class RobotDrive {


    //move forward, backward, move left/right(strafe basically, maybe using distance sensor), turn left, right, rotate(certain amount of degrees),
    // move(either forward, left, or right) for a certain amount of time(timeout thingie)

    public static final double STARTING_ANGLE = 0;
    public static final double FULL_POWER = 1;
    public static final double DEAD_ZONE_THRESHOLD = 0.03;
    public static final double TRIGGER_DIALATION = 0.6;
    public static final double MIN_ROTATING_POWER = 0.25;
    public static final double TEST_POWER = 0.25;
    public static final double ROTATIONAL_DRIFT_CORRECTION = 0.94;

    float currentHeading; //This variable is the current heading of the robot
    Orientation angle; //This variable keeps track of the current heading

    private double COUNTS_PER_INCH = 62.24;  // 89.7158 is the orriginal number
    private double COUNTS_PER_INCH_SIDEWAYS = 67.82;  // 89.7158 is the orriginal number

    public static final double NEVERREST40_ENCODER_CLICKS = 1120;
    public static double INITIAL_HEADING;


    HardwareMap hardwareMap;
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


    //    Servo latchOne;
//    Servo latchTwo;
    revHubIMUGyro revImu;


    public RobotDrive(HardwareMap theHardwareMap, Telemetry theTelemetry) {
        hardwareMap = theHardwareMap;
        //telemetry = theTelemetry;
    }

    public void initImu() {
        revImu = new revHubIMUGyro(hardwareMap, teamUtil.telemetry);

    }

    public void resetHeading() {
        INITIAL_HEADING = revImu.getHeading();
    }

    public double getAbsoluteHeading() {
        return revImu.getAbsoluteHeading();
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


        setAllMotorsWithoutEncoder();

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
        bottomColor = new teamColorSensor(teamUtil.telemetry, bottomColorSensor);
        bottomColor.calibrate();
        //frontRightDistance.setOffset((float)(-3.0));
    }


    public void start() {

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


    public double clip(double power) {

        power = Range.clip(power, -FULL_POWER, FULL_POWER);
        return power;
    }

    public void stopMotors() {
        fRightMotor.setPower(0);
        bRightMotor.setPower(0);
        bLeftMotor.setPower(0);
        fLeftMotor.setPower(0);
    }

    public void driveForward(double power) {
        power = clip(power);
        setAllMotorsWithEncoder();
        fLeftMotor.setPower(power); //neg for F og
        fRightMotor.setPower(power * .875); // pos for F og
        bLeftMotor.setPower(power);  //neg for F og
        bRightMotor.setPower(power * .875); //pos for F og  }

    }

    public void driveBackward(double power) {
        power = clip(power);
        setAllMotorsWithEncoder();
        fLeftMotor.setPower(-power ); //neg for F
        fRightMotor.setPower(-power* .875); // pos for F
        bLeftMotor.setPower(-power );  //neg for F
        bRightMotor.setPower(-power* .875); //pos for F

    }

    public void driveRight(double power) {
        power = clip(power);
        setAllMotorsWithEncoder();
        fLeftMotor.setPower(power);
        fRightMotor.setPower(-power*0.8015384615);
        bLeftMotor.setPower(-power * 0.7361538462);
        bRightMotor.setPower(power *0.8346153846);
    }

    public void driveLeft(double power) {
        power = clip(power);
        setAllMotorsWithEncoder();
        fLeftMotor.setPower(-power );
        fRightMotor.setPower(power*0.9215384615); //0.9615384615
        bLeftMotor.setPower(power * 0.8061538462); //0.8461538462
        bRightMotor.setPower(-power* 0.8846153846);

    }



    public double getDistanceInches(DistanceSensors distanceSensor) {
        double distance = distanceSensor.getDistance();
        if(distance > 20){
            return 1000;
        } else return distance;
    }


    public void distanceTelemetry() {
        teamUtil.telemetry.addData("frontLeftDistance", getDistanceInches(frontLeftDistance));
        teamUtil.telemetry.addData("frontMiddleDistance", frontmiddleDistance.getDistance(DistanceUnit.CM));
        teamUtil.telemetry.addData("frontRightDistance", getDistanceInches(frontRightDistance));
        teamUtil.telemetry.addData("leftDistance", getDistanceInches(leftDistanceSensor));
        teamUtil.telemetry.addData("rightDistance", getDistanceInches(rightDistanceSensor));
        teamUtil.telemetry.addData("backDistance", getDistanceInches(backDistanceSensor));
        teamUtil.telemetry.addLine("front color"+frontmiddleColor.alpha()+":" +frontmiddleColor.red()+":" +frontmiddleColor.green()+":" +frontmiddleColor.blue());

    }

    // Move the robot straight forward or straight backward until the sensor gets to the desired reading
    public void moveToDistance(DistanceSensors sensor, double desiredDistance, double power, long timeOut) {
        teamUtil.log("Moving to Distance");
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        double currentReading = getDistanceInches(sensor);

        // Need to move backwards
        if (currentReading < desiredDistance) {
            while ((getDistanceInches(sensor) < desiredDistance) && teamUtil.keepGoing(timeOutTime)) {
                driveBackward(power);
            }
            // need to move forwards
        } else if (currentReading > desiredDistance) {
            while ((getDistanceInches(sensor) > desiredDistance) && teamUtil.keepGoing(timeOutTime)) {
                driveForward(power);
            }
        }
        stopMotors();
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving to Distance - TIMED OUT!");
        }
        teamUtil.log("Moving to Distance - Finished");

    }

    public void closeToDistanceOr(DistanceSensors frontLeft, DistanceSensors frontRight, double desiredDistance, double power, long timeOut) {
        teamUtil.log("Moving to Distance");
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        double currentReadingLeft = getDistanceInches(frontLeft);
        double currentReadingRight = getDistanceInches(frontRight);

        // Need to move backwards
//        if (currentReading < desiredDistance) {
//            while ((getDistanceInches(sensor) < desiredDistance) && teamUtil.keepGoing(timeOutTime)) {
//                driveBackward(power);
//            }
            // need to move forwards
//        } else
        if (currentReadingLeft > desiredDistance && currentReadingRight > desiredDistance) {
            while (((getDistanceInches(frontLeft) > desiredDistance) && (getDistanceInches(frontRight) > desiredDistance)) && teamUtil.keepGoing(timeOutTime)) {
                driveForward(power);
            }
        }
        stopMotors();
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving to Distance - TIMED OUT!");
        }
        teamUtil.log("Moving to Distance - Finished");

    }

    public void moveToPickUpDistance(long timeOut) {
        teamUtil.log("Moving to Distance");
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        double currentReading = frontmiddleDistance.getDistance(DistanceUnit.CM);

        // Need to move backwards
        if (currentReading < 5.46) {
            while ((frontmiddleDistance.getDistance(DistanceUnit.CM)< 5.46) && teamUtil.keepGoing(timeOutTime)) {
                driveBackward(0.17);
            }
            // need to move forwards
        } else{
            while ((frontmiddleDistance.getDistance(DistanceUnit.CM)> 5.46) && teamUtil.keepGoing(timeOutTime)) {
                driveForward(0.17);
            }
        }
        stopMotors();
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving to Distance - TIMED OUT!");
        }
        teamUtil.log("Moving to Distance - Finished");

    }

    public void driveForward(double power, double timeInMilliseconds) {
        teamUtil.log("Moving Forward Milliseconds: " + timeInMilliseconds);
        ElapsedTime driveTime = new ElapsedTime();
        power = clip(power);
        setBrakeAllDriveMotors();
        do {
            driveForward(power);
        } while (driveTime.milliseconds() < timeInMilliseconds && teamUtil.theOpMode.opModeIsActive());
        stopMotors();
        teamUtil.log("Moving Forward Milliseconds - Finished");
    }

    // move a specified number of inches (using encoder on front right wheel to measure) with the
    // provided power/speed values for each of the 4 motors.
    // This will work in both RUN_USING_ENCODERS and RUN_WITHOUT_ENCODERS but may give very different results...
    public void moveInches(double distance,  double lfspeed, double rfspeed, double lbspeed, double rbspeed, long timeOut){
        teamUtil.log("Move Inches: " + distance + " " + fRightMotor.getMode()); // log the distance and motor mode
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        setBrakeAllDriveMotors();
        int offset = fRightMotor.getCurrentPosition();
        int encoderCounts = (int) (COUNTS_PER_INCH * distance);
        fLeftMotor.setPower(lfspeed);
        fRightMotor.setPower(rfspeed);
        bLeftMotor.setPower(lbspeed);
        bRightMotor.setPower(rbspeed);
        while ((Math.abs(fRightMotor.getCurrentPosition()-offset) < encoderCounts) && teamUtil.keepGoing(timeOutTime)) {
            // cruising to our destination  could log encoder positions here if needed
        }
        stopMotors();
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Move Inches - TIMED OUT!");
        }
        teamUtil.log("Move Inches - Finished");
    }

    public void moveInchesForward(double speed, double inches, long timeOut) {
        teamUtil.log("Moving Inches Forward: " + inches);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        //resets the motors
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorsWithEncoder();
        setBrakeAllDriveMotors();

        //sets the number of desired inches on both motors
        int encoderCounts = (int) (COUNTS_PER_INCH * inches);
        speed = clip(speed);

        do {
//            double driveSpeed = Range.clip( Math.abs(fRightMotor.getCurrentPosition()-encoderCounts)/700, 0.2, 1);
            driveForward(speed);

            teamUtil.log("difference: " + Math.abs(fRightMotor.getCurrentPosition() - encoderCounts));
            teamUtil.log("rightMotorPower: " + fRightMotor.getPower());
            teamUtil.log("fRightMotor: " + getBackLeftMotorPos());
            encoderTelemetry();


        } while ((Math.abs(fRightMotor.getCurrentPosition()) < encoderCounts) && teamUtil.keepGoing(timeOutTime));
        //runs to the set number of inches at the desired speed


        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            encoderTelemetry();
        }

        //turns off both motors
        stopMotors();

        //sets it back to normal
        setAllMotorsWithoutEncoder();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Inches Forward - TIMED OUT!");
        }
        teamUtil.log("Moving Inches Forward - Finished");

    }

    public void moveInchesBackward(double speed, double inches, long timeOut) {
        teamUtil.log("Moving Inches Backward: " + inches);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        //resets the motors
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorsWithEncoder();


        fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //sets the number of desired inches on both motors

        int encoderCounts = (int) (COUNTS_PER_INCH * inches);
        speed = clip(speed);

        do {
            driveBackward(speed);
            teamUtil.log("fRightMotor: " + getBackLeftMotorPos());
            encoderTelemetry();


        } while ((Math.abs(fRightMotor.getCurrentPosition()) < encoderCounts) && teamUtil.keepGoing(timeOutTime));
        //runs to the set number of inches at the desired speed


        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            encoderTelemetry();
        }

        //turns off both motors
        stopMotors();

        //sets it back to normal
        setAllMotorsWithoutEncoder();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Inches Backward - TIMED OUT!");
        }
        teamUtil.log("Moving Inches Backward - Finished");
    }

    public void moveInchesLeft(double speed, double inches, long timeOut) {
        teamUtil.log("Moving Inches Left: " + inches);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        //resets the motors
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorsWithEncoder();


        fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //sets the number of desired inches on both motors

        int encoderCounts = (int) (COUNTS_PER_INCH_SIDEWAYS * inches);
        speed = clip(speed);

        do {
            driveLeft(speed);
            //teamUtil.log("fRightMotor: " + getBackLeftMotorPos());
            encoderTelemetry();


        } while ((Math.abs(fRightMotor.getCurrentPosition()) < encoderCounts) && teamUtil.keepGoing(timeOutTime));
        //runs to the set number of inches at the desired speed


        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            encoderTelemetry();
        }

        //turns off both motors
        stopMotors();

        //sets it back to normal
        setAllMotorsWithoutEncoder();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Inches Left - TIMED OUT!");
        }
        teamUtil.log("Moving Inches Left - Finished");
    }

    public void moveInchesRight(double speed, double inches, long timeOut) {
        teamUtil.log("Moving Inches Right: " + inches);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        //resets the motors
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorsWithEncoder();


        fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //sets the number of desired inches on both motors

        int encoderCounts = (int) (COUNTS_PER_INCH_SIDEWAYS * inches);
        speed = clip(speed);

        do {
            driveRight(speed);
            teamUtil.log("fRightMotor: " + getBackLeftMotorPos());
            encoderTelemetry();


        } while ((Math.abs(fRightMotor.getCurrentPosition()) < encoderCounts) && teamUtil.keepGoing(timeOutTime));
        //runs to the set number of inches at the desired speed


        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            encoderTelemetry();
        }

        //turns off both motors
        stopMotors();

        //sets it back to normal
        setAllMotorsWithoutEncoder();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Inches Right - TIMED OUT!");
        }
        teamUtil.log("Moving Inches Right - Finished");
    }

    public void accelerateInchesForward(double speed, double inches, long timeOut) {

        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = .2;    // NeverRest 40 at 1:1
        final double MAX_ACCEL_PER_INCH = .2; // max power acceleration per inch without skidding
        final double MAX_DECEL_PER_INCH = .075; // max power deceleration per inch without skidding

        int accelerationEncoderCount;
        int decelerationEncoderCount;
        double targetPositionRightMotor = inches * COUNTS_PER_INCH;


        double maxSpeed;
        double speedChange = speed - MIN_SPEED;

        resetAllDriveEncoders();


        if (inches < speedChange / MAX_ACCEL_PER_INCH + speed / MAX_DECEL_PER_INCH) {
            //not enough distance
            moveInchesForward(0.3, inches, timeOut);

        } else {
            teamUtil.log("S: " + speedChange);


            //enough distance to do weird accel curve thingie
            accelerationEncoderCount = (int) ((speedChange / MAX_ACCEL_PER_INCH) * COUNTS_PER_INCH);
            decelerationEncoderCount = (int) ((speedChange / MAX_DECEL_PER_INCH) * COUNTS_PER_INCH);
            double decelerationPoint = targetPositionRightMotor - decelerationEncoderCount;
            maxSpeed = speed;

            teamUtil.log("acceleration distance: " + accelerationEncoderCount * 1 / COUNTS_PER_INCH);
            teamUtil.log("cruising distance: " + (inches - (accelerationEncoderCount + decelerationEncoderCount) * 1 / COUNTS_PER_INCH));
            teamUtil.log("deceleration distance: " + decelerationEncoderCount * 1 / COUNTS_PER_INCH);

            setAllMotorsWithEncoder();
            setBrakeAllDriveMotors();


            while (fRightMotor.getCurrentPosition() < accelerationEncoderCount) {
                //ACCELERATING
                double accelSpeed = (MIN_SPEED + (fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_ACCEL_PER_INCH);
                teamUtil.log("Acceleration speed: " + accelSpeed);
                driveForward(accelSpeed);
            }

            teamUtil.log("start cruising");

            while (fRightMotor.getCurrentPosition() < decelerationPoint) {
                //CRUISING
                driveForward(maxSpeed);
                teamUtil.log("Cruising speed: " + maxSpeed);
            }

            teamUtil.log("start decelerating");

            double initialPosition = fRightMotor.getCurrentPosition();
            while (fRightMotor.getCurrentPosition() - initialPosition < decelerationEncoderCount) {
                //DECELERATING
                double decelSpeed = Range.clip((maxSpeed - ((fRightMotor.getCurrentPosition() - initialPosition) / COUNTS_PER_INCH) * MAX_DECEL_PER_INCH), 0.3, 1);
                teamUtil.log("Deceleration speed: " + decelSpeed);
                teamUtil.log("decelDistanceTraveled: " + (fRightMotor.getCurrentPosition() - initialPosition));
                teamUtil.log("DecelDistance: " + decelerationEncoderCount);

                driveForward(decelSpeed);
            }

            teamUtil.log("stopping motors!!!!");
            stopMotors();
        }
    }

    public void accelerateInchesBackward(double speed, double inches, long timeOut) {

        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = .2;    // NeverRest 40 at 1:1
        final double MAX_ACCEL_PER_INCH = .2; // max power acceleration per inch without skidding
        final double MAX_DECEL_PER_INCH = .075; // max power deceleration per inch without skidding

        int accelerationEncoderCount;
        int decelerationEncoderCount;
        double targetPositionRightMotor = inches * COUNTS_PER_INCH;


        double maxSpeed;
        double speedChange = speed - MIN_SPEED;

        resetAllDriveEncoders();


        if (inches < speedChange / MAX_ACCEL_PER_INCH + speed / MAX_DECEL_PER_INCH) {
            //not enough distance
            moveInchesBackward(0.3, inches, timeOut);

        } else {
            teamUtil.log("S: " + speedChange);


            //enough distance to do weird accel curve thingie
            accelerationEncoderCount = (int) ((speedChange / MAX_ACCEL_PER_INCH) * COUNTS_PER_INCH);
            decelerationEncoderCount = (int) ((speedChange / MAX_DECEL_PER_INCH) * COUNTS_PER_INCH);
            double decelerationPoint = targetPositionRightMotor - decelerationEncoderCount;
            maxSpeed = speed;

            teamUtil.log("acceleration distance: " + accelerationEncoderCount * 1 / COUNTS_PER_INCH);
            teamUtil.log("cruising distance: " + (inches - (accelerationEncoderCount + decelerationEncoderCount) * 1 / COUNTS_PER_INCH));
            teamUtil.log("deceleration distance: " + decelerationEncoderCount * 1 / COUNTS_PER_INCH);
            teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));

            setAllMotorsWithEncoder();
            setBrakeAllDriveMotors();


            while (Math.abs(fRightMotor.getCurrentPosition()) < accelerationEncoderCount) {
                //ACCELERATING
                double accelSpeed = (MIN_SPEED + (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_ACCEL_PER_INCH);
                teamUtil.log("Acceleration speed: " + accelSpeed);
                driveBackward(accelSpeed);
            }

            teamUtil.log("start cruising");

            while (Math.abs(fRightMotor.getCurrentPosition()) < decelerationPoint) {
                //CRUISING
                driveBackward(maxSpeed);
                teamUtil.log("Cruising speed: " + maxSpeed);
            }

            teamUtil.log("start decelerating");

            double initialPosition = Math.abs(fRightMotor.getCurrentPosition());
            while (Math.abs(fRightMotor.getCurrentPosition()) - initialPosition < decelerationEncoderCount) {
                //DECELERATING
                double decelSpeed = Range.clip((maxSpeed - ((Math.abs(fRightMotor.getCurrentPosition()) - initialPosition) / COUNTS_PER_INCH) * MAX_DECEL_PER_INCH), 0.3, 1);
                teamUtil.log("Deceleration speed: " + decelSpeed);
                teamUtil.log("decelDistanceTraveled: " + (Math.abs(fRightMotor.getCurrentPosition()) - initialPosition));
                teamUtil.log("DecelDistance: " + decelerationEncoderCount);

                driveBackward(decelSpeed);
            }

            teamUtil.log("stopping motors!!!!");
            stopMotors();
        }
    }
    public void accelerateInchesLeft(double speed, double inches, long timeOut) {

        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = .35;    // NeverRest 40 at 1:1
        final double MAX_ACCEL_PER_INCH = .2; // max power acceleration per inch without skidding
        final double MAX_DECEL_PER_INCH = .075; // max power deceleration per inch without skidding

        int accelerationEncoderCount;
        int decelerationEncoderCount;
        double targetPositionRightMotor = inches * COUNTS_PER_INCH;


        double maxSpeed;
        double speedChange = speed - MIN_SPEED;

        resetAllDriveEncoders();


        if (inches < speedChange / MAX_ACCEL_PER_INCH + speed / MAX_DECEL_PER_INCH) {
            //not enough distance
            moveInchesLeft(0.3, inches, timeOut);

        } else {
            teamUtil.log("S: " + speedChange);


            //enough distance to do weird accel curve thingie
            accelerationEncoderCount = (int) ((speedChange / MAX_ACCEL_PER_INCH) * COUNTS_PER_INCH);
            decelerationEncoderCount = (int) ((speedChange / MAX_DECEL_PER_INCH) * COUNTS_PER_INCH);
            double decelerationPoint = targetPositionRightMotor - decelerationEncoderCount;
            maxSpeed = speed;

            teamUtil.log("acceleration distance: " + accelerationEncoderCount * 1 / COUNTS_PER_INCH);
            teamUtil.log("cruising distance: " + (inches - (accelerationEncoderCount + decelerationEncoderCount) * 1 / COUNTS_PER_INCH));
            teamUtil.log("deceleration distance: " + decelerationEncoderCount * 1 / COUNTS_PER_INCH);
            teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));

            setAllMotorsWithEncoder();
            setBrakeAllDriveMotors();


            while (Math.abs(fRightMotor.getCurrentPosition()) < accelerationEncoderCount) {
                //ACCELERATING
                double accelSpeed = (MIN_SPEED - (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_ACCEL_PER_INCH);
                teamUtil.log("Acceleration speed: " + accelSpeed);
                driveLeft(accelSpeed);
            }

            teamUtil.log("start cruising");

            while (Math.abs(fRightMotor.getCurrentPosition()) < decelerationPoint) {
                //CRUISING
                driveLeft(maxSpeed);
                teamUtil.log("Cruising speed: " + maxSpeed);
            }

            teamUtil.log("start decelerating");

            double initialPosition = Math.abs(fRightMotor.getCurrentPosition());
            while (Math.abs(fRightMotor.getCurrentPosition()) - initialPosition < decelerationEncoderCount) {
                //DECELERATING
                double decelSpeed = Range.clip((maxSpeed - ((Math.abs(fRightMotor.getCurrentPosition()) - initialPosition) / COUNTS_PER_INCH) * MAX_DECEL_PER_INCH), 0.3, 1);
                teamUtil.log("Deceleration speed: " + decelSpeed);
                teamUtil.log("decelDistanceTraveled: " + (Math.abs(fRightMotor.getCurrentPosition()) - initialPosition));
                teamUtil.log("DecelDistance: " + decelerationEncoderCount);

                driveLeft(decelSpeed);
            }

            teamUtil.log("stopping motors!!!!");
            stopMotors();
        }
    }

    public void accelerateInchesRight(double speed, double inches, long timeOut) {

        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = .35;    // NeverRest 40 at 1:1
        final double MAX_ACCEL_PER_INCH = .2; // max power acceleration per inch without skidding
        final double MAX_DECEL_PER_INCH = .075; // max power deceleration per inch without skidding

        int accelerationEncoderCount;
        int decelerationEncoderCount;
        double targetPositionRightMotor = inches * COUNTS_PER_INCH;


        double maxSpeed;
        double speedChange = speed - MIN_SPEED;

        resetAllDriveEncoders();


        if (inches < speedChange / MAX_ACCEL_PER_INCH + speed / MAX_DECEL_PER_INCH) {
            //not enough distance
            moveInchesRight(0.3, inches, timeOut);

        } else {
            teamUtil.log("S: " + speedChange);


            //enough distance to do weird accel curve thingie
            accelerationEncoderCount = (int) ((speedChange / MAX_ACCEL_PER_INCH) * COUNTS_PER_INCH);
            decelerationEncoderCount = (int) ((speedChange / MAX_DECEL_PER_INCH) * COUNTS_PER_INCH);
            double decelerationPoint = targetPositionRightMotor - decelerationEncoderCount;
            maxSpeed = speed;

            teamUtil.log("acceleration distance: " + accelerationEncoderCount * 1 / COUNTS_PER_INCH);
            teamUtil.log("cruising distance: " + (inches - (accelerationEncoderCount + decelerationEncoderCount) * 1 / COUNTS_PER_INCH));
            teamUtil.log("deceleration distance: " + decelerationEncoderCount * 1 / COUNTS_PER_INCH);
            teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));

            setAllMotorsWithEncoder();
            setBrakeAllDriveMotors();


            while (Math.abs(fRightMotor.getCurrentPosition()) < accelerationEncoderCount) {
                //ACCELERATING
                double accelSpeed = (MIN_SPEED + (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_ACCEL_PER_INCH);
                teamUtil.log("Acceleration speed: " + accelSpeed);
                driveRight(accelSpeed);
            }

            teamUtil.log("start cruising");

            while (Math.abs(fRightMotor.getCurrentPosition()) < decelerationPoint) {
                //CRUISING
                driveRight(maxSpeed);
                teamUtil.log("Cruising speed: " + maxSpeed);
            }

            teamUtil.log("start decelerating");

            double initialPosition = Math.abs(fRightMotor.getCurrentPosition());
            while (Math.abs(fRightMotor.getCurrentPosition()) - initialPosition < decelerationEncoderCount) {
                //DECELERATING
                double decelSpeed = Range.clip((maxSpeed - ((Math.abs(fRightMotor.getCurrentPosition()) - initialPosition) / COUNTS_PER_INCH) * MAX_DECEL_PER_INCH), 0.3, 1);
                teamUtil.log("Deceleration speed: " + decelSpeed);
                teamUtil.log("decelDistanceTraveled: " + (Math.abs(fRightMotor.getCurrentPosition()) - initialPosition));
                teamUtil.log("DecelDistance: " + decelerationEncoderCount);

                driveRight(decelSpeed);
            }

            teamUtil.log("stopping motors!!!!");
            stopMotors();
        }
    }

    public void accelerateToSpeedRight(double startSpeed, double endSpeed){
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = Range.clip(startSpeed, 0.35, 1);    // NeverRest 40 at 1:1
        final double MAX_ACCEL_PER_INCH = .2; // max power acceleration per inch without skidding

        int accelerationEncoderCount;
        double speedChange = Range.clip(endSpeed, 0.35, 1) - MIN_SPEED;

        accelerationEncoderCount = (int) ((speedChange / MAX_ACCEL_PER_INCH) * COUNTS_PER_INCH);

        setAllMotorsWithEncoder();
        resetAllDriveEncoders();

        teamUtil.log("Abbouta accelerate right");
        teamUtil.log("Accel Distance: " + accelerationEncoderCount * 1/COUNTS_PER_INCH);
        teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));
        while (Math.abs(fRightMotor.getCurrentPosition()) < accelerationEncoderCount) {
            //ACCELERATING
            double accelSpeed = (MIN_SPEED + (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_ACCEL_PER_INCH);
            teamUtil.log("Acceleration speed: " + accelSpeed);
            driveRight(accelSpeed);
        }
    }

    public void accelerateToSpeedForwards(double startSpeed, double endSpeed){
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = Range.clip(startSpeed, 0.3, 1);    // NeverRest 40 at 1:1
        final double MAX_ACCEL_PER_INCH = .2; // max power acceleration per inch without skidding

        int accelerationEncoderCount;
        double speedChange = Range.clip(endSpeed, 0.3, 1) - MIN_SPEED;

        accelerationEncoderCount = (int) ((speedChange / MAX_ACCEL_PER_INCH) * COUNTS_PER_INCH);

        setAllMotorsWithEncoder();
        resetAllDriveEncoders();

        teamUtil.log("Abbouta accelerate forward");
        teamUtil.log("Accel Distance: " + accelerationEncoderCount * 1/COUNTS_PER_INCH);
        teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));

        while (Math.abs(fRightMotor.getCurrentPosition()) < accelerationEncoderCount) {
            //ACCELERATING
            double accelSpeed = (MIN_SPEED - (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_ACCEL_PER_INCH);
            teamUtil.log("Acceleration speed: " + accelSpeed);
            driveForward(accelSpeed);
        }
    }

    public void accelerateToSpeedBackwards(double startSpeed, double endSpeed){
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = Range.clip(startSpeed, 0.3, 1);    // NeverRest 40 at 1:1
        final double MAX_ACCEL_PER_INCH = .2; // max power acceleration per inch without skidding

        int accelerationEncoderCount;
        double speedChange = Range.clip(endSpeed, 0.3, 1) - MIN_SPEED;

        accelerationEncoderCount = (int) ((speedChange / MAX_ACCEL_PER_INCH) * COUNTS_PER_INCH);

        setAllMotorsWithEncoder();
        resetAllDriveEncoders();

        teamUtil.log("Abbouta accelerate back");
        teamUtil.log("Accel Distance: " + accelerationEncoderCount * 1/COUNTS_PER_INCH);
        teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));

        while (Math.abs(fRightMotor.getCurrentPosition()) < accelerationEncoderCount) {
            //ACCELERATING
            double accelSpeed = (MIN_SPEED + (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_ACCEL_PER_INCH);
            teamUtil.log("Acceleration speed: " + accelSpeed);
            driveBackward(accelSpeed);
        }
    }

    public void accelerateToSpeedLeft(double startSpeed, double endSpeed){
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = Range.clip(startSpeed, 0.35, 1);    // NeverRest 40 at 1:1
        final double MAX_ACCEL_PER_INCH = .2; // max power acceleration per inch without skidding

        int accelerationEncoderCount;
        double speedChange = Range.clip(endSpeed, 0.35, 1) - MIN_SPEED;

        accelerationEncoderCount = (int) ((speedChange / MAX_ACCEL_PER_INCH) * COUNTS_PER_INCH);

        setAllMotorsWithEncoder();
        resetAllDriveEncoders();

        teamUtil.log("Abbouta accelerate left");
        teamUtil.log("Accel Distance: " + accelerationEncoderCount * 1/COUNTS_PER_INCH);
        teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));

        while (Math.abs(fRightMotor.getCurrentPosition()) < accelerationEncoderCount) {
            //ACCELERATING
            double accelSpeed = (MIN_SPEED - (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_ACCEL_PER_INCH);
            teamUtil.log("Acceleration speed: " + accelSpeed);
            driveLeft(accelSpeed);
        }
    }

///////////////////////////////////////////////////////////////////////////////////////////////////////////
    //deceleration stuff

    public void decelerateInchesRight(double startSpeed, double inches){
        double startingPosition = fRightMotor.getCurrentPosition();
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = 0.35;    // NeverRest 40 at 1:1
        final double MAX_DECEL_PER_INCH = .05; // max power acceleration per inch without skidding

        int decelerationEncoderCount;
        double targetPositionRightMotor = inches * COUNTS_PER_INCH;

        double speedChange = startSpeed - MIN_SPEED;

        decelerationEncoderCount = (int) ((speedChange / MAX_DECEL_PER_INCH) * COUNTS_PER_INCH);
        double endingPoint = startingPosition - inches * COUNTS_PER_INCH;
        double decelerationPoint = endingPoint + decelerationEncoderCount;


        setAllMotorsWithEncoder();
//        resetAllDriveEncoders();

        while (fRightMotor.getCurrentPosition() > decelerationPoint) {
            //CRUISING
            driveRight(startSpeed);
            teamUtil.log("Cruising speed: " + startSpeed);
        }

        teamUtil.log("start decelerating");

        while (fRightMotor.getCurrentPosition() > endingPoint) {
            //DECELERATING
            double decelSpeed = Range.clip((startSpeed - (Math.abs(fRightMotor.getCurrentPosition() - decelerationPoint) / COUNTS_PER_INCH) * MAX_DECEL_PER_INCH), 0.3, 1);
            teamUtil.log("Deceleration speed: " + decelSpeed);
            teamUtil.log("decelDistanceTraveled: " + (Math.abs(fRightMotor.getCurrentPosition() - decelerationPoint)));
            teamUtil.log("DecelDistance: " + decelerationEncoderCount);

            driveRight(decelSpeed);
        }

    }

    public void decelerateInchesLeft(double startSpeed, double inches){
        double startingPosition = fRightMotor.getCurrentPosition();

        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = 0.35;    // NeverRest 40 at 1:1
        final double MAX_DECEL_PER_INCH = .05; // max power acceleration per inch without skidding

        int decelerationEncoderCount;

        double speedChange = startSpeed - MIN_SPEED;

        decelerationEncoderCount = (int) ((speedChange / MAX_DECEL_PER_INCH) * COUNTS_PER_INCH);
        double endingPoint = startingPosition + inches * COUNTS_PER_INCH;
        double decelerationPoint = endingPoint - decelerationEncoderCount;


        setAllMotorsWithEncoder();
//        resetAllDriveEncoders();

        while (fRightMotor.getCurrentPosition() < decelerationPoint) {
            //CRUISING
            driveLeft(startSpeed);
            teamUtil.log("Cruising speed: " + startSpeed);
        }

        teamUtil.log("start decelerating");

        while (fRightMotor.getCurrentPosition() < endingPoint) {
            //DECELERATING
            double decelSpeed = Range.clip((startSpeed - ((fRightMotor.getCurrentPosition() - decelerationPoint) / COUNTS_PER_INCH) * MAX_DECEL_PER_INCH), 0.3, 1);
            teamUtil.log("Deceleration speed: " + decelSpeed);
            teamUtil.log("decelDistanceTraveled: " + (fRightMotor.getCurrentPosition() - decelerationPoint));
            teamUtil.log("DecelDistance: " + decelerationEncoderCount);

            driveLeft(decelSpeed);
        }

    }

    public void decelerateToSpeedLeft(double startSpeed){
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = 0.35;    // NeverRest 40 at 1:1
        final double MAX_DECEL_PER_INCH = .05; // max power acceleration per inch without skidding

        int decelerationEncoderCount;
        double speedChange = startSpeed - MIN_SPEED;

        decelerationEncoderCount = (int) ((speedChange / MAX_DECEL_PER_INCH) * COUNTS_PER_INCH);

        setAllMotorsWithEncoder();
        resetAllDriveEncoders();

        teamUtil.log("Abbouta decelerate left");
        teamUtil.log("decel Distance: " + decelerationEncoderCount * 1/COUNTS_PER_INCH);
        teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));
        while (Math.abs(fRightMotor.getCurrentPosition()) < decelerationEncoderCount) {

            double accelSpeed = (startSpeed + (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_DECEL_PER_INCH);
            teamUtil.log("deceleration speed: " + accelSpeed);
            driveLeft(accelSpeed);
        }
    }


    public void decelerateToSpeedForwards(double startSpeed){
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = 0.35;    // NeverRest 40 at 1:1
        final double MAX_DECEL_PER_INCH = .05; // max power acceleration per inch without skidding

        int decelerationEncoderCount;
        double speedChange = startSpeed - MIN_SPEED;

        decelerationEncoderCount = (int) ((speedChange / MAX_DECEL_PER_INCH) * COUNTS_PER_INCH);

        setAllMotorsWithEncoder();
        resetAllDriveEncoders();

        teamUtil.log("Abbouta decelerate forward");
        teamUtil.log("decel Distance: " + decelerationEncoderCount * 1/COUNTS_PER_INCH);
        teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));
        while (Math.abs(fRightMotor.getCurrentPosition()) < decelerationEncoderCount) {

            double accelSpeed = (startSpeed + (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_DECEL_PER_INCH);
            teamUtil.log("deceleration speed: " + accelSpeed);
            driveForward(accelSpeed);
        }
    }
    public void decelerateToSpeedBackwards(double startSpeed){
        final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
        final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
        final double WHEEL_DIAMETER_INCHES = 3.93701;     // For figuring circumference
        final double COUNTS_PER_INCH =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        final double MIN_SPEED = 0.35;    // NeverRest 40 at 1:1
        final double MAX_DECEL_PER_INCH = .05; // max power acceleration per inch without skidding

        int decelerationEncoderCount;
        double speedChange = startSpeed - MIN_SPEED;

        decelerationEncoderCount = (int) ((speedChange / MAX_DECEL_PER_INCH) * COUNTS_PER_INCH);

        setAllMotorsWithEncoder();
        resetAllDriveEncoders();

        teamUtil.log("Abbouta decelerate back");
        teamUtil.log("decel Distance: " + decelerationEncoderCount * 1/COUNTS_PER_INCH);
        teamUtil.log("AbsRightMotorEncoder: " + Math.abs(fRightMotor.getCurrentPosition()));
        while (Math.abs(fRightMotor.getCurrentPosition()) < decelerationEncoderCount) {

            double accelSpeed = (startSpeed - (-fRightMotor.getCurrentPosition() / COUNTS_PER_INCH) * MAX_DECEL_PER_INCH);
            teamUtil.log("deceleration speed: " + accelSpeed);
            driveBackward(accelSpeed);
        }
    }



    // TODO: COACH-To do a wall follow, you will need a version of this that just powers the motors
    // appropriately and doesn't bother with the encoders.  That would be called from within a loop
    // that is checking the distance to the followed wall and adjusting the desiredHeading
    // proportionally.
    // However, since you have this method, I think you could refactor the implementation of 4 of your MoveInches
    // methods to rely on this instead...but maybe a little later...
    public void driveToHeading(double speed, double inches, double desiredHeading, long timeOut) {
        teamUtil.log("Moving Inches at Heading: Inches: " + inches + " Heading: " + desiredHeading);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        //resets the motors
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        setBrakeAllDriveMotors();
        //sets the number of desired inches on both motors

        int encoderCounts = (int) (COUNTS_PER_INCH * inches);
        float driveSpeed = (float) (clip(speed));

        do {
//            double driveSpeed = Range.clip( Math.abs(fRightMotor.getCurrentPosition()-encoderCounts)/700, 0.2, 1);
            universalJoystick(0, driveSpeed, 0, 1, desiredHeading);

            teamUtil.log("difference: " + Math.abs(fRightMotor.getCurrentPosition() - encoderCounts));
            teamUtil.log("rightMotorPower: " + fRightMotor.getPower());
            teamUtil.log("fRightMotor: " + getBackLeftMotorPos());
            encoderTelemetry();


        } while ((Math.abs(fRightMotor.getCurrentPosition()) < encoderCounts) && teamUtil.keepGoing(timeOutTime));
        //runs to the set number of inches at the desired speed


        while (fLeftMotor.isBusy() && fRightMotor.isBusy() && teamUtil.keepGoing(timeOutTime)) {
            encoderTelemetry();
        }

        //turns off both motors
        stopMotors();

        //sets it back to normal
        setAllMotorsWithoutEncoder();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Inches at Heading - TIMED OUT!");
        }
        teamUtil.log("Moving Inches at Heading - Finished");
    }


    //need to remake encoder methods for that stuff


    public void telemetryDriveEncoders() {
        teamUtil.telemetry.addData("front left:", fLeftMotor.getCurrentPosition());
        teamUtil.telemetry.addData("front right:", fRightMotor.getCurrentPosition());

        teamUtil.telemetry.addData("back left:", bLeftMotor.getCurrentPosition());
        teamUtil.telemetry.addData("back right:", bRightMotor.getCurrentPosition());
    }

    public double adjustAngle(double angle) {

        //assuming imu runs from [0, 360] and angle is added/substracted, adjust it to expected reading
        if (angle >= 360) {
            angle -= 360;
        } else if (angle < 0) {
            angle += 360;
        }
        return angle;


    }


    public double getHeading() {
        return
                revImu.correctHeading(revImu.getHeading() - INITIAL_HEADING);
    }
    public double getOriginalHeading() {
        return
                revImu.getHeading() - INITIAL_HEADING;
    }

    // TODO: COACH-This method needs a "speed" parameter that will provide an upper bound
    //  on the rotation speed so that it can be tuned by the caller
    public void rotateToHeading(double desiredHeading) {

        double startHeading = getHeading();

        teamUtil.log("startAngle: " + startHeading);

        int rotateDirection;
        double rotatePower;
        double tolerance = 1;
        boolean completedRotating;

        double rawChangeInAngle = desiredHeading - getHeading();
        double changeInAngle = Math.abs(adjustAngle(rawChangeInAngle));


        if (changeInAngle <= 180) {
            rotateDirection = 1;
        } else {
            rotateDirection = -1;
        }

        do {
            rotatePower = Range.clip(Math.abs((desiredHeading - getHeading())) / 130, MIN_ROTATING_POWER, 0.5);
            teamUtil.log("rotatePower: " + fLeftMotor.getPower());
            teamUtil.log("difference: " + Math.abs((desiredHeading - getHeading())));
            teamUtil.log("heading : " + getHeading());
//            teamUtil.log("completedRotating: " + completedRotating);


            rotateLeft(rotatePower * rotateDirection);

            if (Math.abs(adjustAngle(desiredHeading - getHeading())) > tolerance) {
                completedRotating = false;
            } else {
                completedRotating = true;
                teamUtil.telemetry.addData("I'm done rotating now", "");
                teamUtil.log("done rotating");
            }

//            teamUtil.telemetry.addData("startHeading", startHeading);
//            teamUtil.telemetry.addData("desiredHeading", desiredHeading);
//
//            teamUtil.telemetry.addData("changeInAngle", changeInAngle);
//            teamUtil.telemetry.addData("rotatingPower", rotatePower);
//            teamUtil.telemetry.addData("direction", rotateDirection);
//            teamUtil.telemetry.update();

        } while (!completedRotating);
        stopMotors();

        teamUtil.log("Rotating - Finished");


    }

    public void rotateTo180(){
        if(getHeading() > 180){
            teamUtil.log("GONNA MOVE RIGHT");
            rotateToHeading180Right();

        } else if (getHeading() < 180){
            teamUtil.log("GONNA MOVE LEFT");
            rotateToHeading180Left();
        }
    }


    public void rotateToHeading180Left(){

        double startHeading = getHeading();
        double rotatePower;

        do {
            rotatePower = Range.clip( Math.abs(180-getHeading())/ 130, MIN_ROTATING_POWER, 0.5);

            rotateLeft(rotatePower);

            teamUtil.log("startHeading: " + startHeading);
            teamUtil.log("daHeading: " + getHeading());
            teamUtil.log("DifferenceInAngle: "+ (180-getHeading()));
            teamUtil.log("rotatePower: " + rotatePower);


        } while(getHeading() < 177.5);

        stopMotors();
        teamUtil.log("I'M DONE");
    }

    public void rotateToHeading180Right(){

        double startHeading = getHeading();
        double rotatePower;

        do {
            rotatePower = Range.clip( Math.abs(180-getHeading())/ 130, MIN_ROTATING_POWER, 0.5);

            rotateRight(rotatePower);

            teamUtil.log("startHeading: " + startHeading);
            teamUtil.log("daHeading: " + getHeading());
            teamUtil.log("DifferenceInAngle: "+ (180-getHeading()));
            teamUtil.log("rotatePower: " + rotatePower);

        } while(getHeading() > 180);

        stopMotors();
        teamUtil.log("I'M DONE");

    }


    public double getRelativeHeading(double pseudoHeading){
        return revImu.correctHeading(adjustAngle(pseudoHeading + getHeading()));
    }

    public void rotateToZero(){
        if(getRelativeHeading(180) < 179.5){
            teamUtil.log("ROTATING LEFT");
            rotateToHeadingZeroLeft();
        } else if(getRelativeHeading(180) > 180.5){
            teamUtil.log("ROTATING RIGHT");
            rotateToHeadingZeroRight();
        }

    }


    public void rotateToHeadingZeroLeft(){
        double startHeading = getHeading();
        teamUtil.log("startHeading: " + startHeading);
        teamUtil.log("getRelativeHeading: " + getRelativeHeading(180));

        double rotatePower;

        do{
            rotatePower = Range.clip( Math.abs(180-getRelativeHeading(180))/ 120, MIN_ROTATING_POWER, 0.5);

            rotateLeft(rotatePower);

        }while(getRelativeHeading(180) < 177.5);

        stopMotors();
        teamUtil.log("I'M DONE");


    }

    public void rotateToHeadingZeroRight(){
        double startHeading = getHeading();

        double rotatePower;

        do{
            rotatePower = Range.clip( Math.abs(180-getRelativeHeading(180))/ 130, MIN_ROTATING_POWER, 0.5);

            rotateRight(rotatePower);

        }while(getRelativeHeading(180) > 182.5);

        stopMotors();
        teamUtil.log("I'M DONE");

    }






    // TODO: COACH-This method needs a "speed" parameter that will provide an upper bound
    //  on the rotation speed so that it can be tuned by the caller
    public void turn(double angle, long timeOut) {
        teamUtil.log("Rotating: " + angle);
        long timeOutTime= System.currentTimeMillis()+timeOut;
        timedOut = false;

        double startHeading = getHeading();
        int rotateDirection;
        double tolerance = 3;
        boolean completedRotating;


        double rawDesiredHeading = startHeading + angle;

        teamUtil.log("START_HEADING: " + startHeading);
        teamUtil.log("RAW_DESIRED_HEADING: " + rawDesiredHeading);

        double changeInAngle = Math.abs(getHeading() - rawDesiredHeading);


        //to make sure that the speed of rotation matches the amount of angle we have to our desired heading
        double rotatePower;


        if (angle > 0) {
            rotateDirection = 1; //CCW
        } else rotateDirection = -1; //CW


        if (Math.abs(getHeading() - rawDesiredHeading) > tolerance) {
            completedRotating = false;
        } else {
            completedRotating = true;
            teamUtil.telemetry.addData("I'M DONE ROTATING", "");
            teamUtil.log("done rotating");
        }

        teamUtil.log("CHANGE_IN_ANGLE: " + changeInAngle);

        teamUtil.log("DIRECTION: " + rotateDirection);

        do {
             rotatePower = Range.clip(Math.abs(getHeading() - rawDesiredHeading) / 130, MIN_ROTATING_POWER, 0.5);
             teamUtil.log("rotatePower: " + fLeftMotor.getPower());
             teamUtil.log("difference: " + Math.abs(getHeading() - rawDesiredHeading));
             teamUtil.log("heading : " + getHeading());
             teamUtil.log("completedRotating: " + completedRotating);


            rotateLeft(rotatePower * rotateDirection);

            if (Math.abs(getHeading() - rawDesiredHeading) > tolerance) {
                completedRotating = false;
            } else {
                completedRotating = true;
                teamUtil.telemetry.addData("I'm done rotating now", "");
                teamUtil.log("done rotating");
            }

//            teamUtil.telemetry.addData("startHeading", startHeading);
//            teamUtil.telemetry.addData("desiredHeading", desiredHeading);
//
//            teamUtil.telemetry.addData("changeInAngle", changeInAngle);
//            teamUtil.telemetry.addData("rotatingPower", rotatePower);
//            teamUtil.telemetry.addData("direction", rotateDirection);
//            teamUtil.telemetry.update();

        } while(!completedRotating && teamUtil.keepGoing(timeOutTime));
        stopMotors();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Rotating - TIMED OUT!");
        }
        teamUtil.log("Rotating - Finished");
    }




    public void rotateLeft(double rotatingPower) {
        double power = clip(rotatingPower);
        fLeftMotor.setPower(-power);
        fRightMotor.setPower(power);
        bLeftMotor.setPower(-power);
        bRightMotor.setPower(power);
    }

    public void rotateRight(double rotatingPower) {
        double power = clip(rotatingPower);
        fLeftMotor.setPower(power);
        fRightMotor.setPower(-power);
        bLeftMotor.setPower(power);
        bRightMotor.setPower(-power);
    }

    public boolean checkJoyStickMovement(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, float rightJoyStickY) {
        float left_x = Math.abs(leftJoyStickX);
        float left_y = Math.abs(leftJoyStickY);
        float right_x = Math.abs(rightJoyStickX);
        float right_y = Math.abs(rightJoyStickY);


        return left_x > DEAD_ZONE_THRESHOLD || left_y > DEAD_ZONE_THRESHOLD || right_x > DEAD_ZONE_THRESHOLD || right_y > DEAD_ZONE_THRESHOLD;

    }


//    public float scalePowerJoystick(float joyStickDistance) {
//
//        //exponential equation obtained from online curve fit with points(x is joystickDistance, y is power:
//        //(0,0), (0.5, 0.3), (0.7, 0.5), (1,1)
//        int sign;
//        if (joyStickDistance > 0) {
//            sign = 1;
//        } else sign = -1;
//
//
//        return (float) (0.9950472 * Math.pow(Math.abs(joyStickDistance), 1.82195) * sign);
//    }



    public void universalJoystick(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, double scaleAmount, double robotHeading){
        double angleInDegrees = robotHeading * Math.PI/180;
        float leftX = leftJoyStickX;
        float leftY = leftJoyStickY;
        float rightX = rightJoyStickX;

        float rotatedLeftX = (float)(Math.cos(angleInDegrees)*leftX - Math.sin(angleInDegrees)*leftY);
        float rotatedLeftY = (float)(Math.sin(angleInDegrees)*leftX + Math.cos(angleInDegrees)*leftY);

        //rotate to obtain new coordinates

        driveJoyStick(rotatedLeftX, rotatedLeftY, rightX, scaleAmount);
    }

    public void driveJoyStick(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, double scaleAmount) {

        //left joystick is for moving, right stick is for rotation

        //RN, for FORWARD motion
        //front left should be negative
        //front right should be negative
        //back left should be negative
        //back right should be negative

//        float leftX = (float)(scalePowerJoystick(leftJoyStickX) * scaleAmount);
//        float leftY = (float)(scalePowerJoystick(leftJoyStickY)*scaleAmount);
        float leftX = (float)((leftJoyStickX) * scaleAmount);
        float leftY = (float)((leftJoyStickY)*scaleAmount);

        float rightX = (float)(rightJoyStickX*0.6*scaleAmount);


        float frontLeft = -(leftY - leftX - rightX); //leftY - leftX - rightX(original prior to reverse)
        float frontRight = (-leftY - leftX - rightX);
        float backRight = (-leftY + leftX - rightX);
        float backLeft = -(leftY + leftX - rightX); //leftY + leftX - rightX(original prior to reverse)

//        teamUtil.telemetry.addData("RIGHTX:", rightX);
//        teamUtil.telemetry.addData("LEFTX:", leftX);
//        teamUtil.telemetry.addData("LEFTY:", leftY);
//
//        teamUtil.telemetry.addData("joystickX:", leftJoyStickX);
//        teamUtil.telemetry.addData("joystickY:", leftJoyStickY);

        fLeftMotor.setPower(frontLeft);
        fRightMotor.setPower(frontRight);
        bRightMotor.setPower(backRight);
        bLeftMotor.setPower(backLeft);


    }

    public void driveTelemetry() {
        teamUtil.telemetry.addData("Front Left Motor:", fLeftMotor.getPower());
        teamUtil.telemetry.addData("Front Right Motor:", fRightMotor.getPower());
        teamUtil.telemetry.addData("Back Left Motor:", bLeftMotor.getPower());
        teamUtil.telemetry.addData("Back Right Motor:", bRightMotor.getPower());
        teamUtil.telemetry.addData("Heading:", getAbsoluteHeading());

    }

    public void encoderTelemetry() {
        teamUtil.telemetry.addData("FL ENCODER POS:", fRightMotor.getCurrentPosition());

    }

    public int getBackLeftMotorPos() {
        return bLeftMotor.getCurrentPosition();
    }

    public int getBackRightMotorPos() {
        return bRightMotor.getCurrentPosition();
    }

    public int getFrontRightMotorPos() {
        return fRightMotor.getCurrentPosition();
    }

    public int getFrontLeftMotorPos() {
        return fLeftMotor.getCurrentPosition();
    }


    //            <goBILDA5202SeriesMotor name="fRightMotor" port="0" /> //backleft
    //            <goBILDA5202SeriesMotor name="bRightMotor" port="1" />
    //            <goBILDA5202SeriesMotor name="fLeftMotor" port="2" />
    //            <goBILDA5202SeriesMotor name="bLeftMotor" port="3" />
}
