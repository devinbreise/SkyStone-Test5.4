package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;


// A class to encapsulate the motion of the 3 servos in the Stone Grabber
public class Grabber {


    public static final double ROTATE_GRABBER_INCREMENT = 0.1;

    //far from the robot
    public final double GRABBER_ONE_STOW_POS = 0.84; //1
    public final double GRABBER_ONE_PICKUP_POS = 0.5;
    public final double GRABBER_ONE_OPEN_POS = 0.29;
    public final double GRABBER_ONE_WIDE_CLOSED_POS = 0.135;
    public final double GRABBER_ONE_SLIGHTLY_OPEN = 0.19;
    public final double GRABBER_ONE_SAFE_ROTATE_POS = 0.075;
    //
    //    //close to the robot
    public final double GRABBER_TWO_CAPSTONE_RELEASE = 0.04-0.05;
    public final double GRABBER_TWO_STOW_POS = 0.13-0.03;
    public final double GRABBER_TWO_PICKUP_POS = 0.5-0.05;
    public final double GRABBER_TWO_OPEN_POS = 0.66-0.05;
    public final double GRABBER_TWO_WIDE_CLOSED_POS = 0.815-0.05;
    public final double GRABBER_TWO_SLIGHTLY_OPEN = 0.76-0.05;
    public final double GRABBER_TWO_SAFE_ROTATE_POS = 0.7-0.05;

    public final double rotate_outside = 0.79;
    public final double rotate_narrow = 0.476;
    public final double rotate_inside = 0.19;

    private Servo grabberOne;
    private Servo grabberTwo;
    private Servo rotateServo;

    public enum GrabberState {
        GRABBER_STOWED,
        GRABBER_NARROW_DROP,
        GRABBER_OPEN,
        GRABBER_CLOSED_WIDE,
        GRABBER_ROTATE_POS,
        GRABBER_PICKUP,
        GRABBER_CAPSTONE,
        GRABBER_UNKNOWN
    }

    public enum GrabberRotation{
        UNKNOWN,
        INSIDE,
        OUTSIDE,
        MIDDLE
    }

    private boolean grabberRunning = false;
    GrabberState grabberState = GrabberState.GRABBER_UNKNOWN;
    public GrabberRotation rotation = GrabberRotation.UNKNOWN;

    Telemetry telemetry;
    HardwareMap hardwareMap;

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public Grabber(HardwareMap theHardwareMap, Telemetry theTelemetry) {
        teamUtil.log ("Constructing Grabber");
        telemetry = theTelemetry;
        hardwareMap = theHardwareMap;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void initGrabber() {
        teamUtil.log ("Initializing Grabber");
        grabberOne = hardwareMap.servo.get("grabberOne");
        grabberTwo = hardwareMap.servo.get("grabberTwo");
        rotateServo = hardwareMap.servo.get("rotateServo");
        grabberState = GrabberState.GRABBER_STOWED;
        rotation = GrabberRotation.UNKNOWN;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void grabberStow(){
        grabberOne.setPosition(GRABBER_ONE_STOW_POS);
        grabberTwo.setPosition(GRABBER_TWO_STOW_POS);
        grabberState = GrabberState.GRABBER_STOWED;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void openGrabber() {
        grabberOne.setPosition(GRABBER_ONE_OPEN_POS);
        grabberTwo.setPosition(GRABBER_TWO_OPEN_POS);
        grabberState = GrabberState.GRABBER_OPEN;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void slightlyOpenGrabber() {
        grabberOne.setPosition(GRABBER_ONE_SLIGHTLY_OPEN);
        grabberTwo.setPosition(GRABBER_TWO_SLIGHTLY_OPEN);
        grabberState = GrabberState.GRABBER_OPEN;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void grabberPickup(){
        grabberOne.setPosition(GRABBER_ONE_PICKUP_POS);
        grabberTwo.setPosition(GRABBER_TWO_PICKUP_POS);
        grabberState = GrabberState.GRABBER_PICKUP;
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void dropCapstone(){
        //grabberOne.setPosition(GRABBER_ONE_STOW_POS);
        grabberTwo.setPosition(GRABBER_TWO_CAPSTONE_RELEASE);
        grabberState = GrabberState.GRABBER_CAPSTONE;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void closeGrabberWide() {
        grabberOne.setPosition(GRABBER_ONE_WIDE_CLOSED_POS);
        grabberTwo.setPosition(GRABBER_TWO_WIDE_CLOSED_POS);
        grabberState = GrabberState.GRABBER_CLOSED_WIDE;

    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void grabberRotatePos() {
        grabberOne.setPosition(GRABBER_ONE_SAFE_ROTATE_POS);
        grabberTwo.setPosition(GRABBER_TWO_SAFE_ROTATE_POS);
        grabberState = GrabberState.GRABBER_ROTATE_POS;

    }

//    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    public void closeGrabberToggle() {
//        if (grabberState == GrabberState.GRABBER_OPEN) {
//            closeGrabberWide();
//        } else if (grabberState == GrabberState.GRABBER_CLOSED_WIDE) {
//            grabberRotatePos();
//        } else if (grabberState == GrabberState.GRABBER_ROTATE_POS) {
//            closeGrabberWide();
//        }
//    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void narrowOpen(){
        grabberOne.setPosition(GRABBER_ONE_SLIGHTLY_OPEN);
        grabberTwo.setPosition(GRABBER_TWO_OPEN_POS);
        grabberState = GrabberState.GRABBER_NARROW_DROP;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void toggleRotate() {
        if(rotateServo.getPosition() == rotate_outside){
            rotateServo.setPosition(rotate_inside);
            rotation = GrabberRotation.INSIDE;
        }else {
            rotateServo.setPosition(rotate_outside);
            rotation = GrabberRotation.OUTSIDE;
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public boolean isSafeToRotate() {
        return grabberState == GrabberState.GRABBER_CLOSED_WIDE || grabberState == GrabberState.GRABBER_ROTATE_POS;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void rotate(Grabber.GrabberRotation newRotation){
        switch(newRotation){
            case INSIDE:
                rotateServo.setPosition(rotate_inside);
                rotation = GrabberRotation.INSIDE;
                break;
            case OUTSIDE:
                rotateServo.setPosition(rotate_outside);
                rotation = GrabberRotation.OUTSIDE;
                break;
            case MIDDLE:
                rotateServo.setPosition(rotate_narrow);
                rotation = GrabberRotation.MIDDLE;
                break;
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /*    public void stowGrabberNoWait(){
        if(grabberRunning == false){
            grabberRunning = true;
            Thread t1 = new Thread(new Runnable(){
                @Override
                public void run(){
                    rotateServo.setPosition(rotate_inside);
                    teamUtil.sleep(1500);
                    grabberOne.setPosition(GRABBER_ONE_STOW_POS);
                    grabberTwo.setPosition(GRABBER_TWO_STOW_POS);
                    grabberRunning = false;
                }
            });
        }
    }

 */

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void grabberTelemetry() {
        teamUtil.telemetry.addData("Grabber State: ", grabberState);
        teamUtil.telemetry.addData("Grabber Rotation: ", rotation);
        teamUtil.telemetry.addData("GrabberOne: ", grabberOne.getPosition());
        teamUtil.telemetry.addData("GrabberTwo: ", grabberTwo.getPosition());
        teamUtil.telemetry.addData("RotateServo: ", rotateServo.getPosition());
    }

}



