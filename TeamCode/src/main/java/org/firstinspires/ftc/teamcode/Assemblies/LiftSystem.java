package org.firstinspires.ftc.teamcode.Assemblies;



import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class LiftSystem {
    Grabber grabber;
    Lift lift;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    enum LiftSystemState{
        IDLE,
        GRAB_AND_STOW,
        DEPLOY_FOR_PICKUP,
    }
    LiftSystemState state = LiftSystemState.IDLE;


    public LiftSystem(HardwareMap theHardwareMap, Telemetry theTelemetry){
        telemetry = theTelemetry;
        hardwareMap = theHardwareMap;

        grabber = new Grabber(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
    }

    public void initLiftSystem(){
        grabber.initGrabber();
        //here we are moving the grabber to a set position where it is out of the way and convienient
        grabber.closeGrabberNarrow();
        teamUtil.sleep(750);
        grabber.rotateInside();
        teamUtil.sleep(750);
        grabber.grabberStow();

        lift.initLift();
        //add limit switch at the top of the lift
        lift.downPosition(.3);
        // need to find init positions for the lift base
    }

    public void grabAndStow(String grabberPos){
        state = LiftSystemState.GRAB_AND_STOW;
        if(grabberPos.equals("narrow")){
            grabber.closeGrabberNarrow();
            teamUtil.log("grabbed");

        }else if(grabberPos.equals("wide")){
            grabber.closeGrabberWide();
            teamUtil.log("grabbed");
        }else{
            return;
        }
        teamUtil.sleep(500);
        teamUtil.log("finnished wait");
        lift.downPositionNoWait(0.3);
        teamUtil.log("going down");
        teamUtil.sleep(1500);
        teamUtil.log("finnished wait");
        grabber.rotateInside();
        teamUtil.log("rotating inside");
        teamUtil.sleep(500);
        teamUtil.log("finnished wait");
        state = LiftSystemState.IDLE;
    }

    public void grabAndStowNoWait (final String grabberPos) {
        if (state == LiftSystemState.IDLE) {
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    grabAndStow(grabberPos);
                }
            });
            thread.start();
        }
    }

    public void prepareToGrab(){
        state = LiftSystemState.DEPLOY_FOR_PICKUP;
        grabber.rotateInside();
        lift.upPositionNoWait(0.7);
        teamUtil.sleep(750);
        grabber.grabberPickup();
        teamUtil.sleep(500);
        state = LiftSystemState.IDLE;
    }

    public void prepareToGrabNoWait() {
        if (state == LiftSystemState.IDLE) {
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    prepareToGrab();
                }
            });
            thread.start();
        }
    }

    public void hoverOverFoundation(int level, Grabber.GrabberRotation rotation){
        grabber.rotate(rotation);
        lift.upPosition(.7);
        lift.goToLevel(level);
    }

    public void hoverOverFoundationNoWait(final int level, final Grabber.GrabberRotation rotation) {
        if (state == LiftSystemState.IDLE) {
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    hoverOverFoundation(level, rotation);
                }
            });
            thread.start();
        }
    }
    public void drop(){
        grabber.openGrabber();
    }
    public void liftDown(){
        lift.goToBottomNoWait();
    }

    public void openGrabber(){
        grabber.openGrabber();
    }

    public void closeGrabberWide(){
        grabber.closeGrabberWide();
    }
}
