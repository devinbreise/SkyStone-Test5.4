package org.firstinspires.ftc.teamcode.Assemblies;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.basicLibs.revHubIMUGyro;

public class RobotDrive {


    //move forward, backward, move left/right(strafe basically, maybe using distance sensor), turn left, right, rotate(certain amount of degrees),
    // move(either forward, left, or right) for a certain amount of time(timeout thingie)

    public static final double STARTING_ANGLE = 0;
    public static final double FULL_POWER = 1;
    public static final double DEAD_ZONE_THRESHOLD = 0.03;
    public static final double TRIGGER_DIALATION = 0.6;
    public static final double MIN_ROTATING_POWER = 0.3;
    public static final double TEST_POWER = 0.25;

    float currentHeading; //This variable is the current heading of the robot
    Orientation angle; //This variable keeps track of the current heading

    private double COUNTS_PER_INCH = 59.4178;  // 89.7158 is the orriginal number

    HardwareMap hardwareMap;
    Telemetry telemetry;
    DcMotor fLeftMotor;
    DcMotor bLeftMotor;
    DcMotor fRightMotor;
    DcMotor bRightMotor;

    //    Servo latchOne;
//    Servo latchTwo;
    BNO055IMU theImu;
    revHubIMUGyro imu;


    public RobotDrive(HardwareMap theHardwareMap, Telemetry theTelemetry) {
        hardwareMap = theHardwareMap;
        telemetry = theTelemetry;
    }

    public void initImu(){
        revHubIMUGyro imu = new revHubIMUGyro(hardwareMap, telemetry, theImu);
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

        setAllMotorsWithoutEncoder();

        //Wise words of Coach, the enlightened one, from book 1:
        //"To make this sort of encoder driven movement both fast and consistent,
        // its usually neccesary to master smooth acceleration and deceleration to minimize wheel slippage.
        // You might want to start thinking about what that code would look like."
    }


    public void initServos() {


    }

    public void initSensors() {


    }


    public void start() {

    }

    public void setAllMotorsWithoutEncoder() {
        fLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runAllMotorsToPosition() {
        fLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setAllTargetPositions(double inches) {
        fLeftMotor.setTargetPosition((int) (COUNTS_PER_INCH * inches));
        fRightMotor.setTargetPosition((int) (COUNTS_PER_INCH * inches));
        bLeftMotor.setTargetPosition((int) (COUNTS_PER_INCH * inches));
        bRightMotor.setTargetPosition((int) (COUNTS_PER_INCH * inches));
    }

    public void setZeroAllDriveMotors() {
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

        power = Range.clip(power, FULL_POWER, -FULL_POWER);
        return power;
    }

    public void stopMotors() {
        fLeftMotor.setPower(0);
        bLeftMotor.setPower(0);
        fRightMotor.setPower(0);
        bRightMotor.setPower(0);
    }

    public void driveForward(double power) {
        power = clip(power);
        fLeftMotor.setPower(-power); //neg for F
        fRightMotor.setPower(power); // pos for F
        bLeftMotor.setPower(-power);  //neg for F
        bRightMotor.setPower(power); //pos for F  }

    }

    public void driveBackward(double power) {
        power = clip(power);
        fLeftMotor.setPower(power); //neg for F
        fRightMotor.setPower(-power); // pos for F
        bLeftMotor.setPower(power);  //neg for F
        bRightMotor.setPower(-power); //pos for F

    }

    public void driveRight(double power) {
        power = clip(power);
        fLeftMotor.setPower(-power);
        fRightMotor.setPower(-power);
        bLeftMotor.setPower(power);
        bRightMotor.setPower(power);
    }

    public void driveLeft(double power) {
        power = clip(power);
        fLeftMotor.setPower(power);
        fRightMotor.setPower(power);
        bLeftMotor.setPower(-power);
        bRightMotor.setPower(-power);

    }

    public void testDriveSlow() {
        fLeftMotor.setPower(-TEST_POWER);
        fRightMotor.setPower(TEST_POWER);
        bLeftMotor.setPower(-TEST_POWER);
        bRightMotor.setPower(TEST_POWER);
    }


//    public void driveForward(double power, int target) {
//
//        power = clip(power);
//        int pos = fLeftMotor.getCurrentPosition();
//
//        do {
//            driveForward(power);
//        } while (pos < target);
//
//        stopMotors();
//    }
//
//    public void driveForward(double power, int target, int maxTime) {
//
//        power = clip(power);
//        int pos = fLeftMotor.getCurrentPosition();
//
//        ElapsedTime motorElapsedTime = new ElapsedTime();
//
//        do {
//            driveForward(power);
//
//        } while (pos < target && motorElapsedTime.milliseconds() < maxTime);
//
//        stopMotors();
//    }

    public void moveInchesForward(double speed, double inches) {
        //resets the motors
        resetAllDriveEncoders();

        setZeroAllDriveMotors();

        //sets the number of desired inches on both motors
        setAllTargetPositions(inches);

        //runs to the set number of inches at the desired speed
        fLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //sets the desired speed on both motors
//            fLeftMotor.setPower(speed);
//            fRightMotor.setPower(speed);
        speed = clip(speed);

//                do {
//            driveForward(speed);
//        } while (pos < target);

        //lets the two moving motors finish the task
        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            // TODO: Add some telemetry output here so we can see what's happening on the driver station phone
        }

        //turns off both motors
        stopMotors();

        //sets it back to normal
        setAllMotorsWithoutEncoder();

    }

    public void moveInchesForward(double speed, double inches, int maxTime) {
        //resets the motors
        resetAllDriveEncoders();
        setZeroAllDriveMotors();
        setAllTargetPositions(inches);

        ElapsedTime motorTime = new ElapsedTime();

        //runs to the set number of inches at the desired speed
        runAllMotorsToPosition();

        //sets the desired speed on both motors
//            fLeftMotor.setPower(speed);
//            fRightMotor.setPower(speed);
        speed = clip(speed);
        driveForward(speed);

        if (motorTime.milliseconds() > maxTime) {
            stopMotors();
        }

        //lets the two moving motors finish the task
        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            // TODO: Add some telemetry output here so we can see what's happening on the driver station phone
        }

        //turns off both motors
        stopMotors();
        //sets it back to normal
        setAllMotorsWithoutEncoder();
    }

    public void moveInchesBackward(double speed, double inches) {
        //resets the motors
        resetAllDriveEncoders();

        setZeroAllDriveMotors();

        //sets the number of desired inches on both motors
        setAllTargetPositions(inches);

        //runs to the set number of inches at the desired speed
        runAllMotorsToPosition();


        //sets the desired speed on both motors
//            fLeftMotor.setPower(speed);
//            fRightMotor.setPower(speed);
        speed = clip(speed);
        driveBackward(speed);

        //lets the two moving motors finish the task
        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            // TODO: Add some telemetry output here so we can see what's happening on the driver station phone
        }

        //turns off both motors
        stopMotors();

        //sets it back to normal
        setAllMotorsWithoutEncoder();

    }

    public void moveInchesBackward(double speed, double inches, int maxTime) {
        //resets the motors
        resetAllDriveEncoders();
        setZeroAllDriveMotors();
        setAllTargetPositions(inches);

        ElapsedTime motorTime = new ElapsedTime();

        //runs to the set number of inches at the desired speed
        runAllMotorsToPosition();

        //sets the desired speed on both motors
//            fLeftMotor.setPower(speed);
//            fRightMotor.setPower(speed);
        speed = clip(speed);
        driveBackward(speed);

        if (motorTime.milliseconds() > maxTime) {
            stopMotors();
        }

        //lets the two moving motors finish the task
        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            // TODO: Add some telemetry output here so we can see what's happening on the driver station phone
        }

        //turns off both motors
        stopMotors();
        //sets it back to normal
        setAllMotorsWithoutEncoder();
    }

    public void moveInchesLeft(double speed, double inches) {
        //resets the motors
        resetAllDriveEncoders();

        setZeroAllDriveMotors();

        //sets the number of desired inches on both motors
        setAllTargetPositions(inches);

        //runs to the set number of inches at the desired speed
        runAllMotorsToPosition();


        //sets the desired speed on both motors
//            fLeftMotor.setPower(speed);
//            fRightMotor.setPower(speed);
        speed = clip(speed);
        driveLeft(speed);

        //lets the two moving motors finish the task
        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            // TODO: Add some telemetry output here so we can see what's happening on the driver station phone
        }

        //turns off both motors
        stopMotors();

        //sets it back to normal
        setAllMotorsWithoutEncoder();

    }

    public void moveInchesLeft(double speed, double inches, int maxTime) {
        //resets the motors
        resetAllDriveEncoders();
        setZeroAllDriveMotors();
        setAllTargetPositions(inches);

        ElapsedTime motorTime = new ElapsedTime();

        //runs to the set number of inches at the desired speed
        runAllMotorsToPosition();

        //sets the desired speed on both motors
//            fLeftMotor.setPower(speed);
//            fRightMotor.setPower(speed);
        speed = clip(speed);
        driveLeft(speed);

        if (motorTime.milliseconds() > maxTime) {
            stopMotors();
        }

        //lets the two moving motors finish the task
        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            // TODO: Add some telemetry output here so we can see what's happening on the driver station phone
        }

        //turns off both motors
        stopMotors();
        //sets it back to normal
        setAllMotorsWithoutEncoder();
    }

    public void moveInchesRight(double speed, double inches) {
        //resets the motors
        resetAllDriveEncoders();

        setZeroAllDriveMotors();

        //sets the number of desired inches on both motors
        setAllTargetPositions(inches);

        //runs to the set number of inches at the desired speed
        runAllMotorsToPosition();


        //sets the desired speed on both motors
//            fLeftMotor.setPower(speed);
//            fRightMotor.setPower(speed);
        speed = clip(speed);
        driveRight(speed);

        //lets the two moving motors finish the task
        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            // TODO: Add some telemetry output here so we can see what's happening on the driver station phone
        }

        //turns off both motors
        stopMotors();

        //sets it back to normal
        setAllMotorsWithoutEncoder();

    }

    public void moveInchesRight(double speed, double inches, int maxTime) {
        //resets the motors
        resetAllDriveEncoders();
        setZeroAllDriveMotors();
        setAllTargetPositions(inches);

        ElapsedTime motorTime = new ElapsedTime();

        //runs to the set number of inches at the desired speed
        runAllMotorsToPosition();

        //sets the desired speed on both motors
//            fLeftMotor.setPower(speed);
//            fRightMotor.setPower(speed);
        speed = clip(speed);
        driveRight(speed);

        if (motorTime.milliseconds() > maxTime) {
            stopMotors();
        }

        //lets the two moving motors finish the task
        while (fLeftMotor.isBusy() && fRightMotor.isBusy()) {
            // TODO: Add some telemetry output here so we can see what's happening on the driver station phone
        }

        //turns off both motors
        stopMotors();
        //sets it back to normal
        setAllMotorsWithoutEncoder();
    }



    public void telemetryDriveEncoders() {
        telemetry.addData("front left:", fLeftMotor.getCurrentPosition());
        telemetry.addData("front right:", fRightMotor.getCurrentPosition());

        telemetry.addData("back left:", bLeftMotor.getCurrentPosition());
        telemetry.addData("back right:", bRightMotor.getCurrentPosition());
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

    public void imuRotateToAngle(double desiredHeading) {

        double startHeading = imu.getHeading();
        int rotateDirection;
        double rotatePower;
        double tolerance = 0.1;
        boolean completedRotating;

        double rawChangeInAngle = desiredHeading - imu.getHeading();
        double changeInAngle = Math.abs(adjustAngle(rawChangeInAngle));


        if (changeInAngle <= 180) {
            rotateDirection = 1;
        } else {
            rotateDirection = -1;
        }

        rotatePower = Range.clip(changeInAngle / 135, MIN_ROTATING_POWER, 1);

        if (changeInAngle > tolerance) {
            completedRotating = false;
        } else {
            completedRotating = true;
            telemetry.addData("I'M DONE ROTATING", "");
        }

        if (!completedRotating) {

            rotateCCW(rotatePower * rotateDirection);
            telemetry.addData("startHeading", startHeading);
            telemetry.addData("desiredHeading", desiredHeading);

            telemetry.addData("changeInAngle", changeInAngle);
            telemetry.addData("rotatingPower", rotatePower);
            telemetry.addData("direction", rotateDirection);
            telemetry.update();

        } else {
            stopMotors();
            return;


        }
    }

    public void imuRotate(double angle) {
        double startHeading = imu.getHeading();
        int rotateDirection;
        double tolerance = 0.1;
        boolean completedRotating;


        double rawDesiredHeading = startHeading + angle;
        double desiredHeading = adjustAngle(rawDesiredHeading);


        double changeInAngle = Math.abs(imu.getHeading() - rawDesiredHeading);

        //to make sure that the speed of rotation matches the amount of angle we have to our desired heading
        double rotatePower = Range.clip(changeInAngle / 135, MIN_ROTATING_POWER, 1);


        if (angle > 0) {
            rotateDirection = 1; //CCW
        } else rotateDirection = -1; //CW


        if (changeInAngle > tolerance) {
            completedRotating = false;
        } else {
            completedRotating = true;
            telemetry.addData("I'M DONE ROTATING", "");
        }

        if (!completedRotating) {

            rotateCCW(rotatePower * rotateDirection);
            telemetry.addData("startHeading", startHeading);
            telemetry.addData("desiredHeading", desiredHeading);

            telemetry.addData("changeInAngle", changeInAngle);
            telemetry.addData("rotatingPower", rotatePower);
            telemetry.addData("direction", rotateDirection);
            telemetry.update();

        } else {
            stopMotors();
            return;

        }


    }

    public void rotateCCW(double rotatingPower) {
        double power = clip(rotatingPower);
        fLeftMotor.setPower(-power);
        fRightMotor.setPower(-power);
        bLeftMotor.setPower(-power);
        bRightMotor.setPower(-power);
    }

    public void rotateCW(double rotatingPower) {
        double power = clip(rotatingPower);
        fLeftMotor.setPower(power);
        fRightMotor.setPower(power);
        bLeftMotor.setPower(power);
        bRightMotor.setPower(power);
    }

    public boolean checkJoyStickMovement(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX, float rightJoyStickY) {
        float left_x = Math.abs(leftJoyStickX);
        float left_y = Math.abs(leftJoyStickY);
        float right_x = Math.abs(rightJoyStickX);
        float right_y = Math.abs(rightJoyStickY);


        return left_x > DEAD_ZONE_THRESHOLD || left_y > DEAD_ZONE_THRESHOLD || right_x > DEAD_ZONE_THRESHOLD || right_y > DEAD_ZONE_THRESHOLD;

    }

    public double scaleMovement(double maxPower, double triggerPressure) {
        //right trigger or left trigger slow-down method

        //idea is have variable drivePower that all the main drive methods use, let drivePower = 1 initially,
        // drivePower = scaleMovement(drivePower, gamepad1_right_trigger); at the top, impacts all the other driving methods

        return maxPower - triggerPressure * TRIGGER_DIALATION; //trigger all the way down makes power 40% of max right now

        //wise words of Coach, the enlightened one, from book 2:
        //This might be a great opportunity for visual feedback from the robot using LEDs
    }

    public float scalePowerJoystick(float joyStickDistance) {

        //exponential equation obtained from online curve fit with points(x is joystickDistance, y is power:
        //(0,0), (0.5, 0.3), (0.7, 0.5), (1,1)
        int sign;
        if (joyStickDistance > 0) {
            sign = 1;
        } else sign = -1;


        return (float) (0.9950472 * Math.pow(Math.abs(joyStickDistance), 1.82195) * sign);
    }

    public void driveJoyStick(float leftJoyStickX, float leftJoyStickY, float rightJoyStickX) {

        //left joystick is for moving, right stick is for rotation

        //RN, for FORWARD motion
        //front left should be negative
        //front right should be negative
        //back left should be negative
        //back right should be negative

        float leftX = scalePowerJoystick(leftJoyStickX);
        float leftY = scalePowerJoystick(leftJoyStickY);
        float rightX = scalePowerJoystick(rightJoyStickX);

//        float leftX = leftJoyStickX;
//        float leftY = leftJoyStickY;
//        float rightX = rightJoyStickX;


        float frontLeft = leftY - leftX + rightX;
        float frontRight = -leftY - leftX + rightX;
        float backRight = -leftY + leftX + rightX;
        float backLeft = leftY + leftX + rightX;

        telemetry.addData("RIGHTX:", rightX);
        telemetry.addData("LEFTX:", leftX);
        telemetry.addData("LEFTY:", leftX);

        telemetry.addData("joystickX:", leftJoyStickX);
        telemetry.addData("joystickY:", leftJoyStickY);

        fLeftMotor.setPower(frontLeft);
        fRightMotor.setPower(frontRight);
        bRightMotor.setPower(backRight);
        bLeftMotor.setPower(backLeft);


    }

    public void driveTelemetry() {
        telemetry.addData("Front Left Motor", fLeftMotor.getPower());
        telemetry.addData("Front Right Motor", fRightMotor.getPower());
        telemetry.addData("Back Left Motor", bLeftMotor.getPower());
        telemetry.addData("Back Right Motor", bRightMotor.getPower());
    }

}
