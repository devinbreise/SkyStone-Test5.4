package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.HiTecServo;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;


public class Grabber {


    public static final double ROTATE_GRABBER_INCREMENT = 0.1;


    //far from the robot
    public final double GRABBER_ONE_STOW_POS = 1; //1
    public final double GRABBER_ONE_OPEN_POS = 0.5;
    public final double GRABBER_ONE_WIDE_CLOSED_POS = 0.3;
    public final double GRABBER_ONE_NARROW_CLOSED_POS = 0.07;

    //close to the robot
    public final double GRABBER_TWO_STOW_POS = 0;
    public final double GRABBER_TWO_OPEN_POS = 0.5;
    public final double GRABBER_TWO_WIDE_CLOSED_POS = 0.7;
    public final double GRABBER_TWO_NARROW_CLOSED_POS = 0.78;

    public final double rotate_outside = 0.79;
    public final double rotate_narrow = 0.476;
    public final double rotate_inside = 0.19;

    private Servo grabberOne;
    private Servo grabberTwo;
    private Servo rotateServo;

    private boolean grabberRunning = false;

    GrabberState grabberState;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    //private Servo rotateGrabber;


    public Grabber(HardwareMap theHardwareMap, Telemetry theTelemetry) {
        telemetry = theTelemetry;
        hardwareMap = theHardwareMap;
    }

    public enum GrabberState {
        GRABBER_STOWED,
        GRABBER_NARROW_DROP,
        GRABBER_OPEN,
        GRABBER_CLOSED_WIDE,
        GRABBER_CLOSED_NARROW
    }

    public void initGrabber() {
        grabberOne = hardwareMap.servo.get("grabberOne");
        grabberTwo = hardwareMap.servo.get("grabberTwo");
        rotateServo = hardwareMap.servo.get("rotateServo");
        //rotateGrabber = hardwareMap.servo.get("rotateGrabber");

        //CURRENT_POS = ROTATE_GRABBER_INITIAL_POS;
        //rotateGrabber.setPosition(CURRENT_POS);


        grabberState = GrabberState.GRABBER_STOWED;
    }
    public void stowGrabber(){
        if(grabberRunning == false){
            grabberRunning = true;
            Thread t1 = new Thread(new Runnable(){
                @Override
                public void run(){
                    rotateServo.setPosition(rotate_inside);
                    teamUtil.sleep(1500);
                    grabberOne.setPosition(GRABBER_ONE_STOW_POS);
                    grabberTwo.setPosition(GRABBER_TWO_STOW_POS);

                }
            });
        }
    }

    public void grabberTelemetry() {
        telemetry.addData("GrabberOne: ", grabberOne.getPosition());
        telemetry.addData("GrabberTwo: ", grabberTwo.getPosition());
        telemetry.addData("GrabberState: ", grabberState);
        telemetry.addData("RotatePosition: ", rotateServo.getPosition());
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

    public void closeGrabberToggle() {
        if (grabberState == GrabberState.GRABBER_OPEN) {
            closeGrabberWide();
        } else if (grabberState == GrabberState.GRABBER_CLOSED_WIDE) {
            closeGrabberNarrow();
        } else if (grabberState == GrabberState.GRABBER_CLOSED_NARROW) {
            closeGrabberWide();
        }
    }

    public void narrowDrop(){
        grabberOne.setPosition(GRABBER_ONE_OPEN_POS);
        grabberTwo.setPosition(GRABBER_TWO_WIDE_CLOSED_POS);
        grabberState = GrabberState.GRABBER_NARROW_DROP;
    }

    public void toggleRotate() {
        if(rotateServo.getPosition() == rotate_outside){
            rotateServo.setPosition(rotate_inside);
        }else {
            rotateServo.setPosition(rotate_outside);
        }
    }

    public void rotateOutside(){
        rotateServo.setPosition(rotate_outside);
    }

    public void rotateInside(){
        rotateServo.setPosition(rotate_inside);
    }

    public void rotateNarrow(){
        rotateServo.setPosition(rotate_narrow);
    }
    public double rotateGetPosition(){
        return rotateServo.getPosition();
    }
    public void rotateSetPosition(double position){
        rotateServo.setPosition(position);
    }


}



