package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Grabber {


    public static final double ROTATE_GRABBER_INCREMENT = 0.1;


    public static double GRABBER_ONE_OPEN_POS = 1;
    public static double GRABBER_ONE_WIDE_CLOSED_POS = 0.64;
    public static double GRABBER_ONE_NARROW_CLOSED_POS = 0.45;

    public static double GRABBER_TWO_OPEN_POS = 0;
    public static double GRABBER_TWO_WIDE_CLOSED_POS = 0.33;
    public static double GRABBER_TWO_NARROW_CLOSED_POS = 0.52;


    private Servo grabberOne;
    private Servo grabberTwo;
    GrabberState grabberState;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    //private Servo rotateGrabber;


    public Grabber(HardwareMap theHardwareMap, Telemetry theTelemetry) {
        telemetry = theTelemetry;
        hardwareMap = theHardwareMap;
    }

    public enum GrabberState{
        GRABBER_OPEN,
        GRABBER_CLOSED_WIDE,
        GRABBER_CLOSED_NARROW
    }

    public void initGrabber() {
        grabberOne = hardwareMap.servo.get("grabberOne");
        grabberTwo = hardwareMap.servo.get("grabberTwo");
        //rotateGrabber = hardwareMap.servo.get("rotateGrabber");

        //CURRENT_POS = ROTATE_GRABBER_INITIAL_POS;
        //rotateGrabber.setPosition(CURRENT_POS);
        grabberOne.setPosition(GRABBER_ONE_OPEN_POS);
        grabberTwo.setPosition(GRABBER_TWO_OPEN_POS);

        grabberState = GrabberState.GRABBER_OPEN;
    }

    public void telemetryGrabber() {
        telemetry.addData("GrabberOne: ", grabberOne.getPosition());
        telemetry.addData("GrabberTwo: ", grabberTwo.getPosition());
        telemetry.addData("GrabberState: ", grabberState);
    }

    public void openGrabber() {
        grabberOne.setPosition(GRABBER_ONE_OPEN_POS);
        grabberTwo.setPosition(GRABBER_TWO_OPEN_POS);
        grabberState = GrabberState.GRABBER_OPEN;

    }

    public void closeGrabberWide() {
        grabberOne.setPosition(GRABBER_ONE_WIDE_CLOSED_POS);
        grabberTwo.setPosition(GRABBER_TWO_WIDE_CLOSED_POS);
        grabberState = GrabberState.GRABBER_CLOSED_WIDE;

    }

    public void closeGrabberNarrow() {
        grabberOne.setPosition(GRABBER_ONE_NARROW_CLOSED_POS);
        grabberTwo.setPosition(GRABBER_TWO_NARROW_CLOSED_POS);
        grabberState = GrabberState.GRABBER_CLOSED_NARROW;

    }

    public void closeGrabberToggle(){
        if(grabberState == GrabberState.GRABBER_OPEN){
            closeGrabberWide();
        } else if(grabberState == GrabberState.GRABBER_CLOSED_WIDE){
            closeGrabberNarrow();
        } else if(grabberState == GrabberState.GRABBER_CLOSED_NARROW){
            closeGrabberWide();
        }
    }

    public void rotateGrabber() {

    }


}



