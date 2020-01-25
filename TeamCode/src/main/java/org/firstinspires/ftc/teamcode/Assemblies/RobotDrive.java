package org.firstinspires.ftc.teamcode.Assemblies;


import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    private boolean frontSensorsOnly = false;

    public static final double NEVERREST40_ENCODER_CLICKS = 1120;
    public static double INITIAL_HEADING;


    HardwareMap hardwareMap;
    //Telemetry telemetry;
    boolean timedOut = false;
    DcMotorEx fLeftMotor;
    DcMotorEx bLeftMotor;
    DcMotorEx fRightMotor;
    DcMotorEx bRightMotor;
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


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Constructors, Initialization, and Telemetry
    // Your class (and all classes representing functional parts of the robot) should have a constructor
    // to set themselves up as well as an "initialize" method that would be called during the robot initialization and
    // possibly a "start" method that would be called just after start and maybe some "shutdown" methods as well.
    // In other words, they should look a bit like a standard OpMode class.
    // We should figure out what our "standard" approach is going to be for acquiring things from the FTC hardwareMap and initialization.
    // For example, should each class representing a particular assembly on the robot (like the drive)
    // reach into the hardwareMap and get the needed objects (and thus encapsulate the names of those things
    // in the configuration file) or should the overall "robot" class do all that work and pass the needed
    // objects down into the assembly level classes?  Or maybe a hybrid approach where all the "names" in the
    // config file are in one place but the assembly classes do the initialization work...Getting a bit ahead here!

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public RobotDrive(HardwareMap theHardwareMap, Telemetry theTelemetry) {
        hardwareMap = theHardwareMap;
        //telemetry = theTelemetry;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initDriveMotors() {
        //fLeftMotor = hardwareMap.dcMotor.get("fLeftMotor");
        //fRightMotor = hardwareMap.dcMotor.get("fRightMotor");
        //bLeftMotor = hardwareMap.dcMotor.get("bLeftMotor");
       // bRightMotor = hardwareMap.dcMotor.get("bRightMotor");
       // fLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // bLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // setAllMotorsWithoutEncoder();  // This might not be the best option at this point...

        fLeftMotor = hardwareMap.get(DcMotorEx.class,"fLeftMotor");
        fRightMotor = hardwareMap.get(DcMotorEx.class,"fRightMotor");
        bLeftMotor = hardwareMap.get(DcMotorEx.class,"bLeftMotor");
        bRightMotor = hardwareMap.get(DcMotorEx.class,"bRightMotor");
        fLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        fLeftMotor.setVelocityPIDFCoefficients(1.5, 0.15,0,14.9);   // These coeffiecients were found using the technique in this doc: https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#
        fRightMotor.setVelocityPIDFCoefficients(1.5, 0.15,0,14.9);  // these are NOT the defaults for these motors
        bLeftMotor.setVelocityPIDFCoefficients(1.5, 0.15,0,14.9);
        bRightMotor.setVelocityPIDFCoefficients(1.5, 0.15,0,14.9);
        setAllMotorsWithEncoder();
        setBrakeAllDriveMotors(); // TODO: Added this with the new movement methods.  this could break existing stuff
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initSensors(boolean frontOnly) {
        frontSensorsOnly = frontOnly;
        frontLeftDistance = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "frontLeftDistance"));
        frontRightDistance = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "frontRightDistance"));
        if (!frontSensorsOnly) {
            leftDistanceSensor = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "leftDistance"));
            rightDistanceSensor = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "rightDistance"));
            backDistanceSensor = new DistanceSensors(hardwareMap.get(Rev2mDistanceSensor.class, "backDistance"));
        }
        frontmiddleDistance = hardwareMap.get(DistanceSensor.class, "frontColorSensor");
        frontmiddleColor = hardwareMap.get(ColorSensor.class, "frontColorSensor");
        bottomColorSensor = hardwareMap.get(ColorSensor.class, "bottomColorSensor");
        bottomColor = new teamColorSensor(teamUtil.telemetry, bottomColorSensor);
        bottomColor.calibrate();
        frontLeftDistance.setOffset((float)(0.0));
        frontRightDistance.setOffset((float)(0.0));
        if (!frontSensorsOnly) {
            leftDistanceSensor.setOffset((float) (0.0));
            rightDistanceSensor.setOffset((float) (0.0));
            backDistanceSensor.setOffset((float) (0.0));
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void start() {
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void distanceTelemetry() {
        teamUtil.telemetry.addData("frontLeftDistance", getDistanceInches(frontLeftDistance));
        teamUtil.telemetry.addData("frontMiddleDistance", frontmiddleDistance.getDistance(DistanceUnit.CM));
        teamUtil.telemetry.addData("frontRightDistance", getDistanceInches(frontRightDistance));
        if (!frontSensorsOnly) {
            teamUtil.telemetry.addData("leftDistance", getDistanceInches(leftDistanceSensor));
            teamUtil.telemetry.addData("rightDistance", getDistanceInches(rightDistanceSensor));
            teamUtil.telemetry.addData("backDistance", getDistanceInches(backDistanceSensor));
        }
        teamUtil.telemetry.addLine("front color"+frontmiddleColor.alpha()+":" +frontmiddleColor.red()+":" +frontmiddleColor.green()+":" +frontmiddleColor.blue());
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void telemetryDriveEncoders() {
        teamUtil.telemetry.addData("front left:", fLeftMotor.getCurrentPosition());
        teamUtil.telemetry.addData("front right:", fRightMotor.getCurrentPosition());

        teamUtil.telemetry.addData("back left:", bLeftMotor.getCurrentPosition());
        teamUtil.telemetry.addData("back right:", bRightMotor.getCurrentPosition());
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveTelemetry() {
        teamUtil.telemetry.addData("Front Left Motor:", fLeftMotor.getPower());
        teamUtil.telemetry.addData("Front Right Motor:", fRightMotor.getPower());
        teamUtil.telemetry.addData("Back Left Motor:", bLeftMotor.getPower());
        teamUtil.telemetry.addData("Back Right Motor:", bRightMotor.getPower());
        teamUtil.telemetry.addData("Heading:", getAbsoluteHeading());

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void encoderTelemetry() { teamUtil.telemetry.addData("FL ENCODER POS:", fRightMotor.getCurrentPosition()); }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getBackLeftMotorPos() {
        return bLeftMotor.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getBackRightMotorPos() {
        return bRightMotor.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getFrontRightMotorPos() {
        return fRightMotor.getCurrentPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public int getFrontLeftMotorPos() {
        return fLeftMotor.getCurrentPosition();
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Adjust readings from Distance Sensors to make sure we are not relying on bad data.
    // This should be added to the teamDistanceSensor class.
    public double getDistanceInches(DistanceSensors distanceSensor) {
        double distance = distanceSensor.getDistance();
        if(distance > 20){ // we don't trust readings above 20 inches with our current sensors
            return 1000;
        } else return distance;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void setAllMotorsWithoutEncoder() {
        fLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void setAllMotorsWithEncoder() {
        fLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void runAllMotorsToPosition() {
        fLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void setBrakeAllDriveMotors() {
        fLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void resetAllDriveEncoders() {
        fLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // IMU and Angle related methods
    // the team IMU class needs to be rewritten to hold all the methods below
    // WARNING!! Our IMUs are currently mounted upside down.  So counter clockwise is positive.  An adjustment for that could be added to
    // the IMU class.  meanwhile, all the code below presumes an upside down rev Hub...

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Get access to the IMU from the hardware map
    public void initImu() {
        revImu = new revHubIMUGyro(hardwareMap, teamUtil.telemetry);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // adjust the given angle to be in the range 0-360.
    public double adjustAngle(double angle) {
        //assuming imu runs from [0, 360] and angle is added/substracted, adjust it to expected reading
        while (angle >= 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // calculate the min degrees between two points on a circle
    // Assumes both degree measurements are 0-360
    public double minDegreeDiff(double a, double b) {
        if (Math.abs(a-b) <= 180) {
            return Math.abs(a-b);
        } else {
            if (a<b)
                return (360-b) + a;
            else
                return (360-a) + b;
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Make the current heading 0.
    public void resetHeading() {
        INITIAL_HEADING = revImu.getHeading();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // return our current heading as a 0 to 360 range.
    public double getHeading() {
        return adjustAngle(revImu.getHeading() - INITIAL_HEADING);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // return the internal IMU heading without our offsets or fixes
    public double getAbsoluteHeading() {
        return revImu.getAbsoluteHeading();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO: This one needs a comment...does it really need to call correctHeading AND adjustAngle?
    public double getRelativeHeading(double pseudoHeading){
        return revImu.correctHeading(adjustAngle(pseudoHeading + getHeading()));
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Basic Motor Power Methods

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public double clip(double power) {
        power = Range.clip(power, -FULL_POWER, FULL_POWER);
        return power;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void stopMotors() {
        fRightMotor.setPower(0);
        bRightMotor.setPower(0);
        bLeftMotor.setPower(0);
        fLeftMotor.setPower(0);
        teamUtil.log("STOP Motors");
    }
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void setMotorPowers(double fl, double fr, double bl, double br) {
        fLeftMotor.setPower(fl);
        fRightMotor.setPower(fr);
        bLeftMotor.setPower(bl);
        bRightMotor.setPower(br);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveForward(double power) {
        power = clip(power);
        setAllMotorsWithEncoder();
        fLeftMotor.setPower(power); //neg for F og
        fRightMotor.setPower(power * .875); // pos for F og
        bLeftMotor.setPower(power);  //neg for F og
        bRightMotor.setPower(power * .875); //pos for F og  }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveBackward(double power) {
        power = clip(power);
        setAllMotorsWithEncoder();
        fLeftMotor.setPower(-power ); //neg for F
        fRightMotor.setPower(-power* .875); // pos for F
        bLeftMotor.setPower(-power );  //neg for F
        bRightMotor.setPower(-power* .875); //pos for F
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveRight(double power) {
        power = clip(power);
        setAllMotorsWithEncoder();
        fLeftMotor.setPower(power);
        fRightMotor.setPower(-power*0.8015384615);
        bLeftMotor.setPower(-power * 0.7361538462);
        bRightMotor.setPower(power *0.8346153846);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveLeft(double power) {
        power = clip(power);
        setAllMotorsWithEncoder();
        fLeftMotor.setPower(-power );
        fRightMotor.setPower(power*0.9215384615); //0.9615384615
        bLeftMotor.setPower(power * 0.8061538462); //0.8461538462
        bRightMotor.setPower(-power* 0.8846153846);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void rotateLeft(double rotatingPower) {
        double power = clip(rotatingPower);
        fLeftMotor.setPower(-power);
        fRightMotor.setPower(power);
        bLeftMotor.setPower(-power);
        bRightMotor.setPower(power);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void rotateRight(double rotatingPower) {
        double power = clip(rotatingPower);
        fLeftMotor.setPower(power);
        fRightMotor.setPower(-power);
        bLeftMotor.setPower(power);
        bRightMotor.setPower(-power);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // return the difference between the current heading and a target heading.  Returns -180 to 180
    public double getHeadingError(double targetAngle) {

        double robotError;

        // calculate heading error in -179 to +180 range  (
        robotError = targetAngle - getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Run the robot forward/or backward maintaining the specified heading by turning proportionally as needed (P = .1)
    public void followHeading(double Heading, double ticsPerSecond) {
        double velocityAdjust = getHeadingError(Heading) * .1 * ticsPerSecond;
        if (ticsPerSecond > 0) {
            fRightMotor.setVelocity(ticsPerSecond*.87+velocityAdjust);
            fLeftMotor.setVelocity(ticsPerSecond-velocityAdjust);
            bRightMotor.setVelocity(ticsPerSecond*.87+velocityAdjust);
            bLeftMotor.setVelocity(ticsPerSecond-velocityAdjust);
        } else {
            fRightMotor.setVelocity(ticsPerSecond*.87-velocityAdjust);
            fLeftMotor.setVelocity(ticsPerSecond+velocityAdjust);
            bRightMotor.setVelocity(ticsPerSecond*.87-velocityAdjust);
            bLeftMotor.setVelocity(ticsPerSecond+velocityAdjust);

        }
        //teamUtil.log("targetHeading: " + Heading + " Heading:" + getHeading() + " Adjust" + velocityAdjust);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Movement methods using Distance Sensors

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Enhanced multi-stage Move to Distance
    // Uses a 3 stage deceleration path and the IMU to hold the given heading.  initialTicsPerSecond should always be positive.
    // robot will move forward or backward as needed to gain the target distance reading on the supplied Distance Sensor
    // Currently this assumes distance sensors are on the front of the robot and postive power moves the robot forward.
    // TODO: Enhance to work with distance sensors on front or back
    public void newMoveToDistance(DistanceSensors sensor, double distance, double initialTicsPerSecond, double heading, boolean moveBackIfNeeded, long timeOut) {
        boolean details = true;
        teamUtil.log("Moving to Distance: "+ distance + " Velocity: "+ initialTicsPerSecond);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        final double maxPower = initialTicsPerSecond;
        final double minPower = 350; // slow enough to be accurate, fast enough to actually move the robot

        double velocity = maxPower;
        double currentDistance  = sensor.getDistance();

        teamUtil.log("Initial Distance Reading: "+ currentDistance);

        // if we are already close enough, leave the robot where it is
        if (Math.abs(currentDistance - distance) <= 1 ){
            teamUtil.log("Already There!  Not Moving...");

        } else if (currentDistance > distance) { // moving forward to target distance
            final double preDriftTarget = distance+.5;
            final double slowThreshold = distance+5;
            final double decelThreshold = slowThreshold+10;
            final double slope = (maxPower-minPower)/(decelThreshold-slowThreshold); // slope for the decel phase
            teamUtil.log("preDriftTarget: "+ preDriftTarget + " slowThreshold: "+ slowThreshold + " decelThreshold: "+ decelThreshold + " slope: "+ slope);
            // Cruise at max speed
            while (currentDistance > decelThreshold && teamUtil.keepGoing(timeOutTime)) {
                followHeading(heading, maxPower);
                if (details) teamUtil.log("CRUISING: Distance:" + currentDistance + " velocity: " + maxPower);
                currentDistance = sensor.getDistance();
            }
            // Decelerate to min speed
            while (currentDistance > slowThreshold && teamUtil.keepGoing(timeOutTime)) {
                velocity = Math.min((currentDistance - slowThreshold) * slope + minPower,initialTicsPerSecond); // decelerate proportionally down to min - don't exceed initial speed
                followHeading(heading, velocity);
                if (details) teamUtil.log("SLOWING: Distance:" + currentDistance + " velocity: " + velocity);
                currentDistance = sensor.getDistance();
            }
            // cruise at minSpeed once we are very close to target
            while (currentDistance > preDriftTarget && teamUtil.keepGoing(timeOutTime)) {
                followHeading(heading, minPower);
                if (details) teamUtil.log("CRAWLING: Distance:" + currentDistance + " velocity: " + minPower);
                currentDistance = sensor.getDistance();
            }
        } else if (moveBackIfNeeded){ // Moving Backwards to a target distance
            final double preDriftTarget = distance-.5;
            final double slowThreshold = distance-5;
            final double decelThreshold = slowThreshold-10;
            final double slope = (maxPower-minPower)/(decelThreshold-slowThreshold); // slope for the decel phase
            // Cruise at max speed
            while (currentDistance < decelThreshold && teamUtil.keepGoing(timeOutTime)) {
                followHeading(heading, -maxPower);
                if (details) teamUtil.log("CRUISING: Distance:" + currentDistance + " velocity: " + -maxPower);
                currentDistance = sensor.getDistance();
            }
            // Decelerate to min speed
            while (currentDistance < slowThreshold && teamUtil.keepGoing(timeOutTime)) {
                velocity = (slowThreshold - currentDistance) * slope + minPower; // decelerate proportionally down to min
                followHeading(heading, -velocity);
                if (details) teamUtil.log("SLOWING: Distance:" + currentDistance + " velocity: " + -velocity);
                currentDistance = sensor.getDistance();
            }
            // cruise at minSpeed once we are very close to target
            while (currentDistance < preDriftTarget && teamUtil.keepGoing(timeOutTime)) {
                followHeading(heading, -minPower);
                if (details) teamUtil.log("CRAWLING: Distance:" + currentDistance + " velocity: " + -minPower);
                currentDistance = sensor.getDistance();
            }
        }
        stopMotors();
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving to Distance - TIMED OUT!");
        }
        teamUtil.log("Finished Move to Distance");
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Basic Movement methods for inches or time
    // These need to be calibrated for actual inches
    // Use at lower speeds or will slippage may compromise accuracy

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Move forward for a specified amouunt of time and then stop
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

            //teamUtil.log("difference: " + Math.abs(fRightMotor.getCurrentPosition() - encoderCounts));
            //teamUtil.log("rightMotorPower: " + fRightMotor.getPower());
            //teamUtil.log("fRightMotor: " + getBackLeftMotorPos());
            encoderTelemetry();


        } while ((Math.abs(fRightMotor.getCurrentPosition()) < encoderCounts) && teamUtil.keepGoing(timeOutTime));
        //runs to the set number of inches at the desired speed


        /* This is not valid code unless you are in RUN_TO_POSITION
        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            encoderTelemetry();
        }
         */

        //turns off both motors
        stopMotors();

        //sets it back to normal (there is nothing normal about running drive motors without encoders...-Coach)
        //setAllMotorsWithoutEncoder();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Inches Forward - TIMED OUT!");
        }
        teamUtil.log("Moving Inches Forward - Finished");

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveInchesBackward(double speed, double inches, long timeOut) {
        teamUtil.log("Moving Inches Backward: " + inches);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        //resets the motors
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorsWithEncoder();


        //fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //sets the number of desired inches on both motors

        int encoderCounts = (int) (COUNTS_PER_INCH * inches);
        speed = clip(speed);

        do {
            driveBackward(speed);
            //teamUtil.log("fRightMotor: " + getBackLeftMotorPos());
            encoderTelemetry();


        } while ((Math.abs(fRightMotor.getCurrentPosition()) < encoderCounts) && teamUtil.keepGoing(timeOutTime));
        //runs to the set number of inches at the desired speed


        /* This is not valid code unless you are in RUN_TO_POSITION
        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            encoderTelemetry();
        }
         */

        //turns off both motors
        stopMotors();

        //sets it back to normal
        //setAllMotorsWithoutEncoder();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Inches Backward - TIMED OUT!");
        }
        teamUtil.log("Moving Inches Backward - Finished");
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveInchesLeft(double speed, double inches, long timeOut) {
        teamUtil.log("Moving Inches Left: " + inches);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        //resets the motors
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorsWithEncoder();


        //fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //sets the number of desired inches on both motors

        int encoderCounts = (int) (COUNTS_PER_INCH_SIDEWAYS * inches);
        speed = clip(speed);

        do {
            driveLeft(speed);
            //teamUtil.log("fRightMotor: " + getBackLeftMotorPos());
            encoderTelemetry();


        } while ((Math.abs(fRightMotor.getCurrentPosition()) < encoderCounts) && teamUtil.keepGoing(timeOutTime));
        //runs to the set number of inches at the desired speed


        /* This is not valid code unless you are in RUN_TO_POSITION
        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            encoderTelemetry();
        }
         */

        //turns off both motors
        stopMotors();

        //sets it back to normal
        //setAllMotorsWithoutEncoder();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Inches Left - TIMED OUT!");
        }
        teamUtil.log("Moving Inches Left - Finished");
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveInchesRight(double speed, double inches, long timeOut) {
        teamUtil.log("Moving Inches Right: " + inches);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        //resets the motors
        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setAllMotorsWithEncoder();


        //fRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //sets the number of desired inches on both motors

        int encoderCounts = (int) (COUNTS_PER_INCH_SIDEWAYS * inches);
        speed = clip(speed);

        do {
            driveRight(speed);
            //teamUtil.log("fRightMotor: " + getBackLeftMotorPos());
            encoderTelemetry();


        } while ((Math.abs(fRightMotor.getCurrentPosition()) < encoderCounts) && teamUtil.keepGoing(timeOutTime));
        //runs to the set number of inches at the desired speed


        /* This is not valid code unless you are in RUN_TO_POSITION
        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            encoderTelemetry();
        }
         */

        //turns off both motors
        stopMotors();

        //sets it back to normal
        //setAllMotorsWithoutEncoder();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Inches Right - TIMED OUT!");
        }
        teamUtil.log("Moving Inches Right - Finished");
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Advanced Movement methods that handle acceleration and deceleration to enable accurate encoder based movement at higher speeds

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Attempt to cover the specified distance at up to the specified speed using smooth acceleration at the start and deceleration at the end
    // this methods assumes the robot is at rest when it starts and will leave the robot at rest.
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Attempt to cover the specified distance at up to the specified speed using smooth acceleration at the start and deceleration at the end
    // this methods assumes the robot is at rest when it starts and will leave the robot at rest.
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Attempt to cover the specified distance at up to the specified speed using smooth acceleration at the start and deceleration at the end
    // this methods assumes the robot is at rest when it starts and will leave the robot at rest.
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Attempt to cover the specified distance at up to the specified speed using smooth acceleration at the start and deceleration at the end
    // this methods assumes the robot is at rest when it starts and will leave the robot at rest.
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Advanced movement methods that handle acceleration or deceleration but presume the caller will handle what comes before or after


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // accelerate from the startSpeed to the endSpeed trying not to slip the wheels.
    // This will leave the robot moving when the method exits
    // Covered distance can be found in the motor encoder counts
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // accelerate from the startSpeed to the endSpeed trying not to slip the wheels.
    // This will leave the robot moving when the method exits
    // Covered distance can be found in the motor encoder counts
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // accelerate from the startSpeed to the endSpeed trying not to slip the wheels.
    // This will leave the robot moving when the method exits
    // Covered distance can be found in the motor encoder counts
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // accelerate from the startSpeed to the endSpeed trying not to slip the wheels.
    // This will leave the robot moving when the method exits
    // Covered distance can be found in the motor encoder counts
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Decelerate from the startSpeed to a stop after traveling the specified distance while trying not to slip the wheels.
    // This assume the robot is moving when it is called
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Decelerate from the startSpeed to a stop after traveling the specified distance while trying not to slip the wheels.
    // This assume the robot is moving when it is called
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Not really sure what this does...needs a comment!
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

     /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Not really sure what this does...needs a comment!
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
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Not really sure what this does...needs a comment!
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

    final double NEW_COUNTS_PER_INCH = 60; // encoder counts per inch on fRightMotor when moving forward or backwards

    public int newInchesToEncoderTics (double inches) {
        return (int) (inches * NEW_COUNTS_PER_INCH);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Ramp the motors smoothly from start velocity to end velocity use maximum acceleration/deceleration rates
    // This will work if both Velocities are positive or negative but not if one is postive and the other negative
    public void newRampMotors (double startVelocity, double endVelocity, double heading, long timeOut) {
        boolean details = false;
        teamUtil.log("New Ramp Motors: Start:"+ startVelocity + " End:"+endVelocity+" heading:"+heading);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        final double MAX_ACCEL_PER_INCH = 440; // max velocity acceleration per inch without skidding
        final double MAX_DECEL_PER_INCH = 200; // max power deceleration per inch without skidding

        // Figure out the distance we need to ramp in encoder tics
        int encoderCount; // always +
        double speedChange = Math.abs(endVelocity) - Math.abs(startVelocity); // + if accelerating or - if decelerating
        if ( Math.abs(endVelocity) >  Math.abs(startVelocity)) {
            encoderCount = newInchesToEncoderTics(Math.abs(speedChange) / MAX_ACCEL_PER_INCH);
        } else {
            encoderCount = newInchesToEncoderTics(Math.abs(speedChange) / MAX_DECEL_PER_INCH);

        }
        double slope = speedChange / encoderCount; // slope for the velocity ramp.  + or -

        setAllMotorsWithEncoder();
        int initialPosition = fRightMotor.getCurrentPosition();

        // Ramp the motors from one speed to the other
        int distanceTraveled = Math.abs(fRightMotor.getCurrentPosition() - initialPosition); // Always +
        while (distanceTraveled < encoderCount && teamUtil.keepGoing(timeOutTime)) {
            double velocity = slope * ((startVelocity>0) ? distanceTraveled : -distanceTraveled) + startVelocity;
            followHeading(heading, velocity);
            if (details) teamUtil.log("Distance Traveled: " + distanceTraveled + " Velocity: " + velocity);
            distanceTraveled = Math.abs(fRightMotor.getCurrentPosition() - initialPosition); // Always +
        }

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("newRampMotors - TIMED OUT!");
        }
        teamUtil.log("newRampMotors - Finished");

    }



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Attempt to cover the specified distance at up to the specified speed using smooth acceleration at the start and deceleration at the end
    // this methods assumes the robot is at rest when it starts and will leave the robot at rest. Works for forward or backwards motion
    // This version uses setVelocity instead of setPower and also attempts to hold the specified heading
    public void newAccelerateInchesBackward(double maxVelocity, double inches, double heading, long timeOut){
        newAccelerateInchesForward(-maxVelocity, inches, heading, timeOut);
    }

    public void newAccelerateInchesForward(double maxVelocity, double inches, double heading, long timeOut) {
        boolean details = false;
        teamUtil.log("newAccelerateInchesForward: velocity:"+ maxVelocity + " Inches:"+inches+" heading:"+heading);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        double MAX_ACCEL_PER_INCH = 440; // max velocity acceleration per inch without skidding (slope with x=inches)
        double MAX_DECEL_PER_INCH = 175; // max power deceleration per inch without skidding (slope with x=inches)
        double START_SPEED = 440;
        double END_SPEED = 300;
        boolean forward = true;

        if (maxVelocity < 0) {
        //    maxVelocity = maxVelocity *-1;
        //    forward = false;
            START_SPEED = START_SPEED * -1;
            END_SPEED = END_SPEED * -1;
        }

        // Figure out the distances for each phase in encoder tics.  These are all + numbers
        int totalEncoderCount = newInchesToEncoderTics(inches);
        int accelerationEncoderCount = Math.abs(newInchesToEncoderTics((maxVelocity-START_SPEED) / MAX_ACCEL_PER_INCH ));
        int decelerationEncoderCount = Math.abs(newInchesToEncoderTics((maxVelocity-END_SPEED) / MAX_DECEL_PER_INCH ));

        // figure out slopes for acceleration and deceleration phases.  Each could be + or -
        double accelerationSlope = (maxVelocity-START_SPEED) / accelerationEncoderCount;
        double decelerationSlope = (maxVelocity-END_SPEED) / decelerationEncoderCount * -1;

        int initialPosition = fRightMotor.getCurrentPosition();
        int target = initialPosition + ((maxVelocity > 0) ? totalEncoderCount : -totalEncoderCount);
        int cruiseStart, decelerationStart;
        if (accelerationEncoderCount+decelerationEncoderCount < totalEncoderCount) {
            // Enough distance to reach maxVelocity
            cruiseStart = initialPosition + ((maxVelocity > 0) ? accelerationEncoderCount : -accelerationEncoderCount) ;
            decelerationStart = target - ((maxVelocity > 0) ? decelerationEncoderCount : -decelerationEncoderCount) ;
        } else {
            // we don't have enough space to ramp up to full speed so calculate the actual maximum velocity
            // by finding the y value of the two ramp lines where they intersect given the maximum distance
            maxVelocity = (-decelerationSlope*totalEncoderCount * accelerationSlope - START_SPEED * decelerationSlope)/(accelerationSlope - decelerationSlope);
            teamUtil.log("Adjusted Max Velocity to:"+ maxVelocity);

            // recompute shortened ramp phases
            accelerationEncoderCount = Math.abs (newInchesToEncoderTics((maxVelocity-START_SPEED) / MAX_ACCEL_PER_INCH ));
            decelerationEncoderCount = Math.abs (newInchesToEncoderTics((maxVelocity-END_SPEED) / MAX_DECEL_PER_INCH ));
            cruiseStart = initialPosition + ((maxVelocity > 0) ? accelerationEncoderCount : -accelerationEncoderCount);
            decelerationStart = target - ((maxVelocity > 0) ? decelerationEncoderCount : -decelerationEncoderCount);
        }
        teamUtil.log("accelerationSlope:"+ accelerationSlope + " decelerationSlope:"+decelerationSlope+" totalEncoderCount:"+totalEncoderCount);
        teamUtil.log("accelerationEncoderCount:"+ accelerationEncoderCount + " decelerationEncoderCount:"+decelerationEncoderCount+" decelerationStart:"+decelerationStart+" totalEncoderCount:"+totalEncoderCount);

        // ramp up
        newRampMotors(START_SPEED, maxVelocity, heading, timeOut);

        // Cruise at Max Velocity
        int currentPosition = fRightMotor.getCurrentPosition();
        if (maxVelocity > 0) { // driving forward
            while (currentPosition < decelerationStart && teamUtil.keepGoing(timeOutTime)) {
                followHeading(heading, maxVelocity);
                if (details) teamUtil.log("Distance Traveled: " + (currentPosition - initialPosition) + "Velocity: " + maxVelocity);
                currentPosition = fRightMotor.getCurrentPosition();
            }
        } else {
            while (currentPosition > decelerationStart && teamUtil.keepGoing(timeOutTime)) {
                followHeading(heading, maxVelocity);
                if (details) teamUtil.log("Distance Traveled: " + (currentPosition - initialPosition) + "Velocity: " + maxVelocity);
                currentPosition = fRightMotor.getCurrentPosition();
            }

        }

        // ramp down
        newRampMotors(maxVelocity, END_SPEED, heading, timeOutTime - System.currentTimeMillis());

        stopMotors();

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("newRampMotors - TIMED OUT!");
        }
        teamUtil.log("newRampMotors - Finished");

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Move the robot at a constant speed in the direction of the desired heading.  DOES THIS WORK?
    // TODO: COACH-To do a wall follow, you will need a version of this that just powers the motors
    // appropriately and doesn't bother with the encoders.  That would be called from within a loop
    // that is checking the distance to the followed wall and adjusting the desiredHeading
    // proportionally.
    // However, since you have this method, I think you could refactor the implementation of 4 of your MoveInches
    // methods to rely on this instead...but maybe a little later...
//    public void driveToHeading(double speed, double inches, double desiredHeading, long timeOut) {
//        teamUtil.log("Moving Inches at Heading: Inches: " + inches + " Heading: " + desiredHeading);
//        long timeOutTime = System.currentTimeMillis() + timeOut;
//        timedOut = false;
//
//        //resets the motors
//        fRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        fRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//
//        setBrakeAllDriveMotors();
//        //sets the number of desired inches on both motors
//
//        int encoderCounts = (int) (COUNTS_PER_INCH * inches);
//        float driveSpeed = (float) (clip(speed));
//
//        do {
////            double driveSpeed = Range.clip( Math.abs(fRightMotor.getCurrentPosition()-encoderCounts)/700, 0.2, 1);
//            universalJoystick(0, driveSpeed, 0, 1, desiredHeading, );
//
//            teamUtil.log("difference: " + Math.abs(fRightMotor.getCurrentPosition() - encoderCounts));
//            teamUtil.log("rightMotorPower: " + fRightMotor.getPower());
//            teamUtil.log("fRightMotor: " + getBackLeftMotorPos());
//            encoderTelemetry();
//
//
//        } while ((Math.abs(fRightMotor.getCurrentPosition()) < encoderCounts) && teamUtil.keepGoing(timeOutTime));
//        //runs to the set number of inches at the desired speed
//
//
//        while (fLeftMotor.isBusy() && fRightMotor.isBusy() && teamUtil.keepGoing(timeOutTime)) {
//            encoderTelemetry();
//        }
//
//        //turns off both motors
//        stopMotors();
//
//        //sets it back to normal
//        setAllMotorsWithoutEncoder();
//
//        timedOut = (System.currentTimeMillis() > timeOutTime);
//        if (timedOut) {
//            teamUtil.log("Moving Inches at Heading - TIMED OUT!");
//        }
//        teamUtil.log("Moving Inches at Heading - Finished");
//    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //Methods for rotating the robot

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO: DOES THIS WORK?
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void rotateTo180(){
        if(getHeading() > 180){
            teamUtil.log("GONNA MOVE RIGHT");
            rotateToHeading180Right();

        } else if (getHeading() < 180){
            teamUtil.log("GONNA MOVE LEFT");
            rotateToHeading180Left();
        }
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void rotateToZero(){
        if(getRelativeHeading(180) < 179.5){
            teamUtil.log("ROTATING LEFT");
            rotateToHeadingZeroLeft();
        } else if(getRelativeHeading(180) > 180.5){
            teamUtil.log("ROTATING RIGHT");
            rotateToHeadingZeroRight();
        }

    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // TODO: DOES THIS METHOD WORK?
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

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // NEW Rotation methods

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Rotate to the desired direction at the maximum speed
    public enum RobotRotation {TOWARDS_FIELD, TOWARDS_DRIVER, TOWARDS_DEPOT, TOWARDS_BUILDING}

    public void newRotateTo(RobotRotation attitude) {
        switch (attitude) {
            case TOWARDS_FIELD:
                newRotateTo(0.0);
                break;
            case TOWARDS_DRIVER:
                newRotateTo(180.0);
                break;
            case TOWARDS_DEPOT:
                if (teamUtil.alliance == teamUtil.Alliance.RED) {
                    newRotateTo(90.0);
                } else {
                    newRotateTo(270.0);
                }
                break;
            case TOWARDS_BUILDING:
                if (teamUtil.alliance == teamUtil.Alliance.BLUE) {
                    newRotateTo(90.0);
                } else {
                    newRotateTo(270.0);
                }
        }
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Rotate to the desired heading at the maximum speed, slowing for accuracy at the end
    public void newRotateTo(double heading) {
        teamUtil.log("Starting to rotate to " + heading);
        final double decelThreshold = 60; // start deceleration this many degrees from the target
        final double slowThreshold = 10; // slow down to a very slow turn this far from the target
        final double maxPower = 1;
        final double minPower = .15;
        final double decelSlope = (maxPower-minPower)/(decelThreshold-slowThreshold); // + slope
        final double driftDegrees = 1; // cut the motors completely when we are within this many degrees of the target to allow for a little drift
        double leftRotatePower = 1; // Keep track of which way we are rotating
        double rightRotatePower = 1;
        double rotatePower = maxPower; // start at full power

        double currentHeading = getHeading();
        double initialHeading = currentHeading; // Stash this so we can make this a "relative" turn from a heading of 0.

        // Determine how many degrees we need to turn from our current position to get to the target
        double turnDegrees =  minDegreeDiff(heading, currentHeading); // always +
        turnDegrees=turnDegrees-driftDegrees; // stop early to allow for drift

        // Determine which we we are spinning (take the short way around)
        if (currentHeading < heading) {
            if (heading-currentHeading < 180) {
                leftRotatePower = -1;
            } else {
                rightRotatePower = -1;
            }
        } else {
            if (currentHeading-heading < 180) {
                rightRotatePower = -1;
            } else {
                leftRotatePower = -1;
            }
        }
        teamUtil.log("Turn Degrees: " + turnDegrees + " LeftPower: " + leftRotatePower);
        teamUtil.log("Initial Heading: " + initialHeading + " current Heading: " + currentHeading);

        // Number of degrees we have turned (in either direction) since we started
        currentHeading = minDegreeDiff(getHeading() ,initialHeading); // always +

        // Rotate at max power until we get to deceleration phase
        while (currentHeading < turnDegrees - decelThreshold) {
            setMotorPowers(maxPower*leftRotatePower, maxPower*rightRotatePower, maxPower*leftRotatePower, maxPower*rightRotatePower);
            //teamUtil.log("MAX: Relative Heading:"+currentHeading+" DifferenceInAngle: "+ (turnDegrees-currentHeading)+" RotatePower: " + maxPower);
            currentHeading = minDegreeDiff(getHeading() ,initialHeading); // always +
        }

        // rotate at decelerating power as we close to target
        while (currentHeading < turnDegrees - slowThreshold){
            rotatePower = (turnDegrees -slowThreshold - currentHeading)*decelSlope+minPower; // decelerate proportionally down to min
            setMotorPowers(rotatePower*leftRotatePower, rotatePower*rightRotatePower, rotatePower*leftRotatePower, rotatePower*rightRotatePower);
            //teamUtil.log("DECEL: Relative Heading:"+currentHeading+" DifferenceInAngle: "+ (turnDegrees-currentHeading)+" RotatePower: " + rotatePower);
            currentHeading = minDegreeDiff(getHeading() ,initialHeading); // always +
        }

        // rotate at minSpeed once we are very close to target
        while (currentHeading < turnDegrees){
            setMotorPowers(minPower*leftRotatePower, minPower*rightRotatePower, minPower*leftRotatePower, minPower*rightRotatePower);
            //teamUtil.log("CRAWL: Relative Heading:"+currentHeading+" DifferenceInAngle: "+ (turnDegrees-currentHeading)+" RotatePower: " + minPower);
            currentHeading = minDegreeDiff(getHeading() ,initialHeading); // always +
        }
        stopMotors();
        teamUtil.log("Finished Turning.  Actual Heading: "+ getHeading());
    }


    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // NEW Auto Position Methods

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Assumes one of the two front distance sensors is within 9 inches of the foundation and the other one is NOT
    // It will strafe right or left as needed to line up.

    public void newPositionToFoundation(double heading, long timeOut){
        boolean details = false;
        teamUtil.log("newPositionToFoundation: heading:"+ heading);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        double MAX_DISTANCE_FOR_AUTO_DROPOFF = 9;
        double MANEUVERING_DISTANCE = 4;

        if (frontRightDistance.getDistance() < 9) { // we need to strafe right
            newMoveToDistance(frontRightDistance,MANEUVERING_DISTANCE, 1500, heading,false, timeOutTime-System.currentTimeMillis());
            while((frontLeftDistance.getDistance() > MAX_DISTANCE_FOR_AUTO_DROPOFF) && teamUtil.keepGoing(timeOutTime)) {
                driveRight(0.35);
                if (details) teamUtil.log("Front Left Distance:"+frontLeftDistance.getDistance());
            }
            moveInchesRight(0.35, 1, timeOutTime-System.currentTimeMillis());
            newRotateTo(heading);
            newMoveToDistance(frontRightDistance,1.5, 1500, heading,true, timeOutTime-System.currentTimeMillis());

        } else { // We need to strafe left
            newMoveToDistance(frontLeftDistance, MANEUVERING_DISTANCE, 1500, 0, false, timeOutTime-System.currentTimeMillis());
            while ((frontRightDistance.getDistance() > MAX_DISTANCE_FOR_AUTO_DROPOFF) && teamUtil.keepGoing(timeOutTime)) {
                driveLeft(0.35);
                if (details) teamUtil.log("Front Right Distance:"+frontRightDistance.getDistance());
            }
            moveInchesLeft(0.35, 1, timeOutTime-System.currentTimeMillis());
            newRotateTo(heading);
            newMoveToDistance(frontLeftDistance, 1.5, 1500, heading, true, timeOutTime-System.currentTimeMillis());
        }

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("newPositionToFoundation - TIMED OUT!");
        }
        teamUtil.log("newPositionToFoundation - Finished");

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void universalJoystick(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, double scaleAmount, double robotHeading, double heldHeading){
        double angleInDegrees = robotHeading * Math.PI/180; // TODO: Isn't this converting TO Radians resulting in an 'angleInRadians'?
        float leftX = leftJoyStickX;
        float leftY = leftJoyStickY;
        float rightX = rightJoyStickX;

        float rotatedLeftX = (float)(Math.cos(angleInDegrees)*leftX - Math.sin(angleInDegrees)*leftY);
        float rotatedLeftY = (float)(Math.sin(angleInDegrees)*leftX + Math.cos(angleInDegrees)*leftY);

        //rotate to obtain new coordinates

        driveJoyStick(rotatedLeftX, rotatedLeftY, rightX, scaleAmount, heldHeading);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void driveJoyStick(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, double scaleAmount, double heldHeading) {

        float leftX = leftJoyStickX;
        float leftY = leftJoyStickY;
        float rotationAdjustment = rightJoyStickX;

        //left joystick is for moving, right stick is for rotation

        //RN, for FORWARD motion
        //front left should be negative
        //front right should be negative
        //back left should be negative
        //back right should be negative

//        float leftX = (float)(scalePowerJoystick(leftJoyStickX) * scaleAmount);
//        float leftY = (float)(scalePowerJoystick(leftJoyStickY)*scaleAmount);

//        float leftX = (float)((leftJoyStickX) * scaleAmount);
//        float leftY = (float)((leftJoyStickY)*scaleAmount);


        if((leftJoyStickX > -0.1 && leftJoyStickX < 0.1)){
            leftX = 0;
        } else if((leftJoyStickX > -0.9 && leftJoyStickX < 0.9)) {
            leftX = (leftJoyStickX > 0 ? (float)(leftJoyStickX*0.1/0.8 + 0.18) : (float)(leftJoyStickX*4/8 - 0.155));
        } else{
            leftX = (float)(0.8*Math.pow(leftJoyStickX, 11)  + ((leftJoyStickX > 0) ? 0.2 : -0.2)); //0.2 is minimum driving power we intend\
        }

        if((leftJoyStickY> -0.1 && leftJoyStickY < 0.1)){
            leftY = 0;
        } else if((leftJoyStickY > -0.9 && leftJoyStickY < 0.9)) {
            leftY = (leftJoyStickY > 0 ? (float)(leftJoyStickY*0.1/0.8 + 0.18) : (float)(leftJoyStickY*4/8 - 0.155));
        } else{
            leftY = (float)(0.8*Math.pow(leftJoyStickY, 11)  + ((leftJoyStickY > 0) ? 0.2 : -0.2)); //0.2 is minimum driving power we intend\
        }


        rotationAdjustment = (float) (rightJoyStickX * 0.6 * scaleAmount);

//HOLDING HEADING CODE:
//        float frontLeft = -(leftY - leftX ); //leftY - leftX - rightX(original prior to reverse)
//        float frontRight = (-leftY - leftX );
//        float backRight = (-leftY + leftX );
//        float backLeft = -(leftY + leftX ); //leftY + leftX - rightX(original prior to reverse)

//        teamUtil.log("LeftX: " + leftX + " LeftJoystickX: " + leftJoyStickX);
//        teamUtil.log("LeftY: " + leftY + " LeftJoystickY: ");
//        if(Math.abs(rightJoyStickX) > 0.1) {
//            rotationAdjustment = (float) (rightJoyStickX * 0.6 * scaleAmount); // TODO: if rightJoyStickX  is in the dead range, we could use a passed in heading to hold instead...
//        } else {
//            rotationAdjustment = getHeadingError(heldHeading) * .1 * Math.max(Math.max(frontLeft, frontRight), Math.max(backRight, backLeft));
//        }
//
//        frontLeft-=rotationAdjustment;
//        frontRight+=rotationAdjustment;
//        backRight-=rotationAdjustment;
//        backLeft+=rotationAdjustment;



            float frontLeft = -(leftY - leftX - rotationAdjustment); //leftY - leftX - rightX(original prior to reverse)
            float frontRight = (-leftY - leftX - rotationAdjustment);
            float backRight = (-leftY + leftX - rotationAdjustment);
            float backLeft = -(leftY + leftX - rotationAdjustment); //leftY + leftX - rightX(original prior to reverse)

//        teamUtil.telemetry.addData("RIGHTX:", rightX);
//        teamUtil.telemetry.addData("LEFTX:", leftX);
//        teamUtil.telemetry.addData("LEFTY:", leftY);
//
//        teamUtil.telemetry.addData("joystickX:", leftJoyStickX);
//        teamUtil.telemetry.addData("joystickY:", leftJoyStickY);

        fLeftMotor.setPower(frontLeft);
        fRightMotor.setPower(frontRight * 0.9);
        bRightMotor.setPower(backRight * 0.9);
        bLeftMotor.setPower(backLeft);


    }



    //            <goBILDA5202SeriesMotor name="fRightMotor" port="0" /> //backleft
    //            <goBILDA5202SeriesMotor name="bRightMotor" port="1" />
    //            <goBILDA5202SeriesMotor name="fLeftMotor" port="2" />
    //            <goBILDA5202SeriesMotor name="bLeftMotor" port="3" />
}
