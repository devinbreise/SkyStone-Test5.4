package org.firstinspires.ftc.teamcode.TestCode.CoachCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class coachGyro {

    private HardwareMap hardwareMap;
    // The IMU sensor object (integrated imu in the Rev Hub)
    private BNO055IMU imu;
    // IMU State
    Orientation anglesCurrent;
    Orientation anglesLast;
    // our normalized heading
    float currentHeading;

    coachGyro(HardwareMap map, String deviceName) {
        hardwareMap = map;
        // set up our IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled      = true;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, deviceName);
        imu.initialize(parameters);

    }

    float resetHeading() {
        currentHeading = 0;
        anglesLast = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return currentHeading;
    }

    float getHeading(){
        anglesCurrent = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // if we were near the cutoff and appear to have turned through it to the right
        if ((anglesLast.firstAngle >= 90) && (anglesCurrent.firstAngle < 0)) {
            currentHeading = currentHeading + (180 - anglesLast.firstAngle) + (anglesCurrent.firstAngle + 180);
        // if we were near the cutoff and appear to have turned through it to the left
        } else if ((anglesLast.firstAngle < -90) && (anglesCurrent.firstAngle > 0)) {
            currentHeading = currentHeading - (180 - anglesCurrent.firstAngle) - (anglesLast.firstAngle + 180);

        } else { // otherwise we haven't oved through the cutoff and can just do the math
            currentHeading = currentHeading + (anglesCurrent.firstAngle - anglesLast.firstAngle);
        }
        anglesLast = anglesCurrent;
        return currentHeading;
    }
}



