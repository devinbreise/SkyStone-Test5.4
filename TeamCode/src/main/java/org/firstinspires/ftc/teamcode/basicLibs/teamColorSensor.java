package org.firstinspires.ftc.teamcode.basicLibs;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class teamColorSensor {
    private ColorSensor colorSensor;
    private Telemetry telemetry;
    private int matValueRed;
    private int matValueBlue;

    public teamColorSensor(Telemetry theTelemetry, ColorSensor theColorSensor) {
        telemetry = theTelemetry;
        colorSensor = theColorSensor;
    }

    public void detectColor() {


    }

    public void calibrate() {//Call this when light sensor are not on tape
        matValueBlue=colorSensor.blue();
        matValueRed=colorSensor.red();
    }

    public boolean isOnTape(){
        if (onBlue()||onRed()) {
            return true;
        } else {
            return false;
        }

    }
    public boolean onBlue() {
        if (colorSensor.blue()>matValueBlue*1.5) {
            return true;
        } else {
            return false;
        }
    }
    public boolean onRed() {
        if (colorSensor.red()>matValueBlue*1.5) {
            return true;
        } else {
            return false;
        }


    }
    public int redValue() {
        return colorSensor.red();
    }





    public int blueValue() {
        return colorSensor.blue();
    }





}







