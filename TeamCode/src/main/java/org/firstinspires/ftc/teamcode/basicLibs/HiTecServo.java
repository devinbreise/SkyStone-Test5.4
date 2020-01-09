package org.firstinspires.ftc.teamcode.basicLibs;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HiTecServo {
    private Servo theServo;
    private Telemetry telemetry;

    public HiTecServo(Servo aServo, Telemetry aTelemetry ){
        theServo = aServo;
        telemetry = aTelemetry;
    }

    public void goTo (int degrees){
        if (theServo == null){
            teamUtil.telemetry.addData("ERROR", "you have not initialized");
            return;
        }
        if (degrees < 0 || degrees > 150){
            teamUtil.telemetry.addData("ERROR", "Servo degrees out of range");
            return;
        }
        double servoPos = 0.0066666666667 * degrees;
        if(servoPos < 0.3 || servoPos > 0.7){
            teamUtil.telemetry.addData("ERROR", "Servo degrees out of range");
            return;
        } else theServo.setPosition(servoPos);

    }
}
