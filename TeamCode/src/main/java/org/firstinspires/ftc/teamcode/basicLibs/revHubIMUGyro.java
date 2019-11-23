package org.firstinspires.ftc.teamcode.basicLibs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


//This code gives you a number of the current heading of the robot.
// The code tells you how much the robot has moved from its last heading.
// This code can be used for turning until a certain amount of degrees or angle.
// Which can be very useful in the autonomous, or even in the teleop period which lining up your robot.
//For the robot right is postitive and left is negitive
//You can use it by starting from reset heading to 0 and then turn untill less than, greater than or equal to the heading you want
public class revHubIMUGyro {


    private Telemetry telemetry; //This variable repersent the telemetry
    private HardwareMap hardwareMap;
    private BNO055IMU imu; //This variable is the imu
    Orientation angle; //This variable keeps track of the current heading



//OLD VARIABLES FOR OLD CODE--depended on in some files here or there--(CAN BE FOUND WAY BELOW)
    Orientation anglesLast; //This variable keeps track of the current heading
    Orientation anglesCurrent;
    float currentHeading;


    public revHubIMUGyro( HardwareMap hardwaremap, Telemetry telemetry){
        // set up our IMU
        //These are the parameters that the imu uses in the code to name and keep track of the data
        this.telemetry = telemetry;
        this.hardwareMap = hardwaremap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }


//    public revHubIMUGyro() {
//        telemetry = theTelemetry;
//        hardwareMap = theHardwareMap;
//        imu = theimu;
//        currentDirection = 0;
//
//        // set up our IMU
//        //These are the parameters that the imu uses in the code to name and keep track of the data
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//
//
//        imu = hardwareMap.get(BNO055IMU.class, deviceName);
//        imu.initialize(parameters);
//    }


    public void resetHeading(){
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = 0;
        //ideally, you shouldnt have to reset it when moving into teleop, so you only want to do this to right itself if necessary
    }

    public float getAbsoluteHeading() {
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angle.firstAngle;
    }

    public double correctHeading(double angle){
        if(angle < 0){
            return angle+360;
        } else return angle;
    }


    public double getHeading(){
        double angle = getAbsoluteHeading();
        return correctHeading(angle);
    }


//    //This resets the imu current heading of the robot
    public float resetHeading(boolean usingOld) {
        if(usingOld){
            currentHeading = 0;
            anglesLast = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //The code resets the current heading to 0 and keep wwhat the current angle is as you reset it as angles last to
            //use in the code in the future
            return currentHeading;

        }
        else return 0;
    }


    public float getHeading(boolean usingOld) {
        if(usingOld){        anglesCurrent = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("IMU Heading", anglesCurrent.firstAngle);
            // if we were near the cutoff between the positive and negative angles
            // and appear to have turned between the positive and negative sides to the right
            // than see if the last amgle is greater than or equal to 90 and the current angle is less than 0 than do
            // the math to figure out how much degrees the robot has turned between the postive and negitive numbers
            if ((anglesLast.firstAngle >= 90) && (anglesCurrent.firstAngle < 0)) {
                currentHeading = currentHeading + (180 - anglesLast.firstAngle) + (anglesCurrent.firstAngle + 180);
                // if we were near the cutoff and between the positive and negitive angles
                // and appear to have turned between the positive and negitive sides to the left
                //than see if the last angle is less than -90 and the current angle is greater than 0 than do
                //the math to figure out how much degress the robot has turned between the postive and negitive numbers
            } else if ((anglesLast.firstAngle < -90) && (anglesCurrent.firstAngle > 0)) {
                currentHeading = currentHeading - (180 - anglesCurrent.firstAngle) - (anglesLast.firstAngle + 180);

            } else { // if the angles has not jumped between the positive and negitive than just so the math to subtract the current
                //angle and the last angle to find the number between these values to find the current heading of the IMU
                // Note: The IMU heading gets larger to the left and smaller to the right.

                currentHeading = currentHeading - (anglesCurrent.firstAngle - anglesLast.firstAngle);
            }
            anglesLast = anglesCurrent;

            return currentHeading;
        } else return 0;
    }

//    public float getAbsoluteHeading() {
//        anglesCurrent = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        return anglesCurrent.firstAngle;
//    }




}
