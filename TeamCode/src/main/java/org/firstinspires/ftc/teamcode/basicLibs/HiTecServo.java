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
            telemetry.addData("ERROR", "you have not initialized");
            return;
        }
        if (degrees < 0 || degrees > 150){
            telemetry.addData("ERROR", "Servo degrees out of range");
            return;
        }
        theServo.setPosition(0.0066666666667 * degrees);

    }
}
