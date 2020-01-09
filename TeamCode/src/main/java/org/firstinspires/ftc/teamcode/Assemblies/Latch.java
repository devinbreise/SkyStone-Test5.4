package org.firstinspires.ftc.teamcode.Assemblies;

import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Latch {

    //NEED TO INTRODUCE STATE MACHINE(MAKE SURE GRABBER ONE WORKS, THEN COPY-PASTE

    public static final double LATCH_ONE_UP = 0.3;
    public static final double LATCH_ONE_PUSHBOT = 0.59;
    public static final double LATCH_ONE_MIDDLE = 0.64;
    public static final double LATCH_ONE_DOWN = 0.72;

    public static final double LATCH_TWO_UP = 0.73;
    public static final double LATCH_TWO_PUSHBOT = 0.38;
    public static final double LATCH_TWO_MIDDLE = 0.31;
    public static final double LATCH_TWO_DOWN = 0.23;


    public boolean latchIsUp;


    Servo latchOne;
    Servo latchTwo;
    Telemetry telemetry;
    HardwareMap hardwareMap;


    public Latch(HardwareMap theHardwareMap, Telemetry theTelemetry) {
        hardwareMap = theHardwareMap;
        telemetry = theTelemetry;
    }

    public void initLatch() {
        latchOne = hardwareMap.servo.get("latchOne");
        latchTwo = hardwareMap.servo.get("latchTwo");
        latchMiddle();
        latchIsUp = true;
    }

    public void toggleLatch() {
        if (latchIsUp) {
            latchDown();
            latchIsUp = false;
        } else {
            latchUp();
            latchIsUp = true;
        }
    }

    public void latchUp() {
        latchOne.setPosition(LATCH_ONE_UP);
        latchTwo.setPosition(LATCH_TWO_UP);
        teamUtil.telemetry.addData("latch is up", "");
    }


    public void latchDown() {
        latchOne.setPosition(LATCH_ONE_DOWN);
        latchTwo.setPosition(LATCH_TWO_DOWN);
        teamUtil.telemetry.addData("latch is down", "");
    }

    public void latchMiddle() {
        latchOne.setPosition(LATCH_ONE_MIDDLE);
        latchTwo.setPosition(LATCH_TWO_MIDDLE);
        teamUtil.telemetry.addData("latch is in middle position", "");
    }

    public void latchPushbot() {
        latchOne.setPosition(LATCH_ONE_PUSHBOT);
        latchTwo.setPosition(LATCH_TWO_PUSHBOT);
        teamUtil.telemetry.addData("latch is in pushbot position", "");
    }

    public void latchTelemetry() {
        teamUtil.telemetry.addData("latchOne: ", latchOne.getPosition());
        teamUtil.telemetry.addData("latchTwo: ", latchTwo.getPosition());
        teamUtil.telemetry.addData("latchup?", latchIsUp);

    }


}
