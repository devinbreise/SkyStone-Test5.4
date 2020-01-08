package org.firstinspires.ftc.teamcode.basicLibs;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class goBuildAServo2000 {
    private Servo theServo;
    private Telemetry telemetry;

    public goBuildAServo2000(Servo aServo, Telemetry aTelemetry ){
        theServo = aServo;
        telemetry = aTelemetry;
    }

    public void goTo (int degrees){
        if (theServo == null){
            teamUtil.telemetry.addData("ERROR", "you have not initialized");
            return;
        }
        if (degrees < 0 || degrees > 250){
            teamUtil.telemetry.addData("ERROR", "Servo degrees out of range");
            return;
        }
        theServo.setPosition(0.004 * degrees);

    }
}
