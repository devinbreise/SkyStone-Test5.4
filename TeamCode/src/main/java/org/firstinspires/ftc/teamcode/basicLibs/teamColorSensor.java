package org.firstinspires.ftc.teamcode.basicLibs;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class teamColorSensor {
    private ColorSensor colorSensor;
    private Telemetry telemetry;
    private int matValueRed;
    private int matValueBlue;
    private int SkystoneBlack;

    public teamColorSensor(Telemetry theTelemetry, ColorSensor theColorSensor) {
        telemetry = theTelemetry;
        colorSensor = theColorSensor;
    }

    public boolean isSkystone(float reading){
        return reading < 72;
    }



    public float getReading(){
        return colorSensor.alpha();
    }

    public float getGreen(){
        return colorSensor.green();
    }

    public void calibrate() {//Call this when light sensor are not on tape
        matValueBlue=colorSensor.blue();
        matValueRed=colorSensor.red();
    }

    public boolean isOnTape(){
        return onBlue() || onRed();

    }
    public boolean onBlue() {
        return colorSensor.blue() > matValueBlue * 1.5;
    }
    public boolean onRed() {
        return colorSensor.red() > matValueBlue * 1.5;
    }
    public int redValue() {
        return colorSensor.red();
    }





    public int blueValue() {
        return colorSensor.blue();
    }





}







