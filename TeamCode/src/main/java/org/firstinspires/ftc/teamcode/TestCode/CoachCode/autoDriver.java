package org.firstinspires.ftc.teamcode.TestCode.CoachCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.basicLibs.revHubIMUGyro;
import org.firstinspires.ftc.teamcode.basicLibs.teamColorSensor;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;


public class autoDriver {

    private HardwareMap hardwareMap;
    private LinearOpMode opMode;
    private Telemetry telemetry;
    private revHubIMUGyro gyro;
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private teamColorSensor leftColor;
    private teamColorSensor rightColor;
    private DistanceSensor left2m;
    private DistanceSensor right2m;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    private float[] hsvValuesLeft = {0F, 0F, 0F};
    private float[] hsvValuesRight = {0F, 0F, 0F};
    // values is a reference to the hsvValues array.
    private final float[] valuesLeft = hsvValuesLeft;
    private final float[] valuesRight = hsvValuesRight;
    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    private final double COLOR_SCALE_FACTOR = 255;

    autoDriver(HardwareMap map, LinearOpMode mode, Telemetry tel, DcMotor left, DcMotor right, revHubIMUGyro g, teamColorSensor lC, teamColorSensor rC, DistanceSensor l2m, DistanceSensor r2m){
        hardwareMap = map;
        opMode = mode;
        telemetry = tel;
        leftMotor = left;
        rightMotor = right;
        gyro = g;
        leftColor = lC;
        rightColor = rC;
        left2m = l2m;
        right2m = r2m;
    }

    void accelerateForSeconds (double startSpeed, double endSpeed, double seconds) {
        ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        runtime.reset();
        while (runtime.milliseconds() < seconds*1000) {
            double targetSpeed = startSpeed + (endSpeed-startSpeed)/seconds * (runtime.milliseconds()/1000);
            leftMotor.setPower(targetSpeed);
            rightMotor.setPower(targetSpeed);
        }
    }

    void accelerateForEncoderClicks (double startSpeed, double endSpeed, int encoderClicks) {

        int startEncoder = leftMotor.getCurrentPosition();
        int endEncoder = startEncoder+encoderClicks;

        while (leftMotor.getCurrentPosition() < endEncoder) {
            double targetSpeed = startSpeed + (endSpeed-startSpeed)/encoderClicks * (leftMotor.getCurrentPosition()-startEncoder);
            leftMotor.setPower(targetSpeed);
            rightMotor.setPower(targetSpeed);
        }
    }

    // move a specified number of inches
    void smoothMoveInches(double speed, double inches){

        if (opMode.opModeIsActive()) {

            final double COUNTS_PER_MOTOR_REV = 1120;    // NeverRest 40 at 1:1
            final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
            final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
            final double COUNTS_PER_INCH =
                    (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

            final double MIN_SPEED = .2;    // NeverRest 40 at 1:1
            final double MAX_ACCEL_RATE = .1; // max power acceleration per inch without skidding
            final double MAX_DECEL_RATE = .1; // max power deceleration per inch without skidding
            int newLeftTarget;
            int newRightTarget;
            int decelerationPoint;
            int accelerationEncoderCount;
            int decelerationEncoderCount;

            double maxSpeed = speed; // our maximum speed during the whole move
            double speedChange = speed - MIN_SPEED; // the maximum (ideal) amount of acceleration and deceleration

            // Ensure that the opmode is still active
            if (opMode.opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                newLeftTarget = leftMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                newRightTarget = rightMotor.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
                if (inches < speedChange / MAX_ACCEL_RATE + speed / MAX_DECEL_RATE) { // not enough distance to accelerate and decelerate fully
                    accelerationEncoderCount = 100;
                    decelerationEncoderCount = 100;
                    decelerationPoint = newLeftTarget - decelerationEncoderCount;
                    maxSpeed = .3;

                } else { // we can get to cruising speed
                    accelerationEncoderCount = (int) (speedChange / MAX_ACCEL_RATE * COUNTS_PER_INCH);
                    decelerationEncoderCount = (int) (speedChange / MAX_DECEL_RATE * COUNTS_PER_INCH);
                    decelerationPoint = newLeftTarget - decelerationEncoderCount;
                    maxSpeed = speed;
                }
                teamUtil.log("leftTarget:" + newLeftTarget);
                teamUtil.log("accelerationEncoderCount:" + accelerationEncoderCount);
                teamUtil.log("decelerationEncoderCount:" + decelerationEncoderCount);
                teamUtil.log("decelerationPoint:" + decelerationPoint);

                //leftMotor.setTargetPosition(newLeftTarget);
                //rightMotor.setTargetPosition(newRightTarget);

                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                // accelerate smoothly
                teamUtil.log("Accelarating: left Position:" + leftMotor.getCurrentPosition());
                accelerateForEncoderClicks(MIN_SPEED, maxSpeed, accelerationEncoderCount);

                // continue motion at full speed
                teamUtil.log("Cruising: left Position:" + leftMotor.getCurrentPosition());

                while (opMode.opModeIsActive() && leftMotor.getCurrentPosition() < decelerationPoint) {
                }

                // decelerate smoothly
                int clicksToGo = newLeftTarget - leftMotor.getCurrentPosition();
                teamUtil.log("Decelerating: leftPosition:" + leftMotor.getCurrentPosition());
                teamUtil.log("Decelerating: clicksToGo:" + clicksToGo);
                accelerateForEncoderClicks(maxSpeed, MIN_SPEED, clicksToGo);

                // Stop all motion;
                teamUtil.log("stopping motors: leftPosition:" + leftMotor.getCurrentPosition());
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                //leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                //  sleep(250);   // optional pause after each move
            }
        }
    }

    // move a specified number of inches
    void moveInches(double speed, double inches){

         final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // NeverRest 40 at 1:1
         final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
         final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
         final double     COUNTS_PER_INCH         =
                (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newRightTarget = rightMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            leftMotor.setTargetPosition(newLeftTarget);
            rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the liftLoop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the liftLoop test.
            while (opMode.opModeIsActive() &&
                    (leftMotor.isBusy() && rightMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        leftMotor.getCurrentPosition(),
                        rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    // turn # of degrees at specified speed.  negative degrees is left
    void turn(double speed, float degrees) {
        float currentAngle = gyro.resetHeading(true);
        float goalAngle = currentAngle + degrees;
        // turn until we have made it
        if (degrees < 0) { // turning left
            leftMotor.setPower(0);
            rightMotor.setPower(speed);
            while ((currentAngle > goalAngle) && (opMode.opModeIsActive())) {
                telemetry.addData("turning left", currentAngle);
                currentAngle = gyro.getHeading(true);
                telemetry.update();
            }
        } else { // turning right
            leftMotor.setPower(speed);
            rightMotor.setPower(0);
            while ((currentAngle < goalAngle) && (opMode.opModeIsActive())) {
                telemetry.addData("turning right", currentAngle);
                currentAngle = gyro.getHeading(true);
                telemetry.update();
            }
        }
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

    void setZeroHeading () {
        gyro.resetHeading();
    }

    public void motorsOn(double speed) {
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
    }
    public void motorsOff() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    // turn to absolute heading
    // heading is in FTC coordinate system (e.g. 0 degrees is 12 o'clock, 90 degrees is 9 o'clock, etc.  360 and 0 are equal)
    void spinLeftTo (float newHeading) {
        float originalHeading = normalizeHeading(gyro.getAbsoluteHeading());
        if (newHeading >= 360) {newHeading = newHeading -360;}
        newHeading = newHeading - 8;
        teamUtil.log("originalHeading:"+originalHeading);
        teamUtil.log("newHeading:"+newHeading);
        if (opMode.opModeIsActive()) {
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftMotor.setPower(-.2);
            rightMotor.setPower(.45);
            // now spin until we reach our goal
            while (opMode.opModeIsActive() && !reachedSpinLeftTarget(originalHeading, newHeading, normalizeHeading(gyro.getAbsoluteHeading()))){
                }
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }
    }

    boolean reachedSpinLeftTarget(float originalHeading, float targetHeading, float currentHeading) {
        if (originalHeading < targetHeading) {
            return (currentHeading >= targetHeading);
        } else {
            return ((currentHeading >= targetHeading) && ((currentHeading-targetHeading) <10) && ((currentHeading-targetHeading) >=0));
        }
    }

    // convert to a 0-360 coordinate system
    float normalizeHeading(float heading) {
        if (heading < 0) {
            heading = 360+heading;
        }
        return heading;
    }

    // turn # of degrees at specified speed.  negative degrees is left
    void spinTo (float heading) {
        if (opMode.opModeIsActive()) {
            double decelAngle;
            float currentAngle = gyro.getHeading(true);
            float goalAngle = heading;
            if (goalAngle < currentAngle) {
                decelAngle = currentAngle - .75 * (currentAngle - goalAngle);
                goalAngle = goalAngle + 8;
            } else {
                decelAngle = currentAngle + .75 * (goalAngle - currentAngle);
                goalAngle = goalAngle - 5;
            }
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // turn until we have made it
            if (goalAngle < currentAngle) { // spinning left
                leftMotor.setPower(-.2);
                rightMotor.setPower(.4);
                while (opMode.opModeIsActive() && (currentAngle > goalAngle) && (opMode.opModeIsActive())) {
                    telemetry.addData("turning left", currentAngle);
                    if (currentAngle < decelAngle) {
                        leftMotor.setPower(-.2);
                        rightMotor.setPower(.4);
                    }
                    currentAngle = gyro.getHeading(true);
                    telemetry.update();
                }
            } else { // spining right
                leftMotor.setPower(.4);
                rightMotor.setPower(-.2);
                while (opMode.opModeIsActive() && (currentAngle < goalAngle) && (opMode.opModeIsActive())) {
                    telemetry.addData("turning right", currentAngle);
                    if (currentAngle > decelAngle) {
                        leftMotor.setPower(.4);
                        rightMotor.setPower(-.2);
                    }
                    currentAngle = gyro.getHeading(true);
                    telemetry.update();
                }
                rightMotor.setPower(0);
                leftMotor.setPower(0);
            }
        }
    }

    // turn # of degrees at specified speed.  negative degrees is left
    void spin (double speed, float degrees) {
        float currentAngle = gyro.resetHeading(true);
        float goalAngle = currentAngle + degrees;
        if (degrees<0){
            goalAngle=goalAngle+8;
        } else {
            goalAngle=goalAngle-5;
        }
        double decelAngle = currentAngle + degrees*.75; //decelerate during last 25% of turn
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // turn until we have made it
        if (degrees < 0) { // spinning left
            leftMotor.setPower(-.2);
            rightMotor.setPower(.4);
            while ((currentAngle > goalAngle) && (opMode.opModeIsActive())) {
                telemetry.addData("turning left", currentAngle);
                if (currentAngle < decelAngle) {
                    leftMotor.setPower(-.2);
                    rightMotor.setPower(.4);
                }
                currentAngle = gyro.getHeading(true);
                telemetry.update();
            }
        } else { // spining right
            leftMotor.setPower(.4);
            rightMotor.setPower(-.2);
            while ((currentAngle < goalAngle) && (opMode.opModeIsActive())) {
                telemetry.addData("turning right", currentAngle);
                if (currentAngle > decelAngle) {
                    leftMotor.setPower(.4);
                    rightMotor.setPower(-.2);
                }
                currentAngle = gyro.getHeading(true);
                telemetry.update();
            }
            rightMotor.setPower(0);
            leftMotor.setPower(0);
        }
    }

    void rightWaitForLine() {
        while (opMode.opModeIsActive() && !rightColor.isOnTape()) {}
    }

    void squareOnLine(double speed) {
         double     COUNTS_PER_INCH = 89.7158;
         double     WHEEL_BASE = 16;

        ElapsedTime runtime = new ElapsedTime();
        int leftEncoder = 0;
        int rightEncoder = 0;
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
        runtime.reset();
        while ((leftEncoder == 0 || rightEncoder == 0) && runtime.seconds()<3) {
            teamUtil.log("sensing for line...");
            if (leftEncoder == 0 && leftColor.isOnTape()) {
                leftEncoder = leftMotor.getCurrentPosition();
                teamUtil.log("Left color sensor on tape at : " + leftEncoder);
            }
            if (rightEncoder == 0 && rightColor.isOnTape()) {
                rightEncoder = rightMotor.getCurrentPosition();
                teamUtil.log("Right color sensor on tape at : " + rightEncoder);
            }
        }
        float turnAngle = (float)-Math.toDegrees(Math.atan((rightEncoder-leftEncoder)/(WHEEL_BASE*COUNTS_PER_INCH)));
        teamUtil.log("Spin Degrees: " + turnAngle);
        if (speed > 0) {
            spin(.3, turnAngle);
        } else {
            spin(.3, -turnAngle);
        }
    }
/*    void squareOnBlueLine(double speed) {
        updateColorSensorData();
        int blackLeft = leftColor.blue();
        int blackRight = rightColor.blue();
        leftMotor.setPower(speed);
        rightMotor.setPower(speed);
        while ((leftColor.blue() < blackLeft *2) || (rightColor.blue() < blackRight *2)) {
            if (leftColor.blue() < blackLeft *2) {
                leftMotor.setPower(speed);
            } else {
                leftMotor.setPower(0);
            }
            if (rightColor.blue() < blackLeft *2) {
                rightMotor.setPower(speed);
            } else {
                rightMotor.setPower(0);
            }
        }
    }
*/
    boolean squareOnWall(double speed) {

        if ((left2m.getDistance(DistanceUnit.INCH) > 50 ) || (right2m.getDistance(DistanceUnit.INCH) > 50)) {
            return false;
        }
        if (left2m.getDistance(DistanceUnit.INCH) > right2m.getDistance(DistanceUnit.INCH)) {
            leftMotor.setPower(speed);
            rightMotor.setPower(-speed);
            while (left2m.getDistance(DistanceUnit.INCH) > right2m.getDistance(DistanceUnit.INCH)) {
                // wait until we are square
            }
        } else {
            leftMotor.setPower(-speed);
            rightMotor.setPower(speed);
            while (right2m.getDistance(DistanceUnit.INCH) > left2m.getDistance(DistanceUnit.INCH)) {
                // wait until we are square
            }
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);
        return true;
    }
}