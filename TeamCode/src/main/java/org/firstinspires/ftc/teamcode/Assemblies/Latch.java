package org.firstinspires.ftc.teamcode.Assemblies;


import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Latch {


    public static final double LATCH_ONE_DOWN = 0.7;
    public static final double LATCH_TWO_DOWN = 0.7;
    public static final double LATCH_TWO_UP = 0.5;
    public static final double LATCH_ONE_UP = 0.5;
    public boolean latchIsUp;


    Servo latchOne;
    Servo latchTwo;
    Telemetry telemetry;
    HardwareMap hardwareMap;


    public Latch(HardwareMap theHardwareMap, Telemetry theTelemetry) {
        hardwareMap = theHardwareMap;
        telemetry = theTelemetry;

    }

    public void initializeLatch() {
        latchOne = hardwareMap.servo.get("latchOne");
        latchTwo = hardwareMap.servo.get("latchTwo");
        latchOne.setPosition(LATCH_ONE_UP);
        latchTwo.setPosition(LATCH_TWO_UP);
        latchIsUp = true;


    }

    public void toggleLatch(){
        if(latchIsUp){
            latchDown();
        } else latchUp();
    }

    public void latchUp() {
        latchOne.setPosition(LATCH_ONE_UP);
        latchTwo.setPosition(LATCH_TWO_UP);
        telemetry.addData("latch is up", "");

    }


    public void latchDown() {
        latchOne.setPosition(LATCH_ONE_DOWN);
        latchTwo.setPosition(LATCH_TWO_DOWN);
        telemetry.addData("latch is down", "");

    }

    public void telemetryLatch(){
        telemetry.addData("latchOne: ", latchOne.getPosition());
        telemetry.addData("latchTwo: ", latchTwo.getPosition());

    }


}
