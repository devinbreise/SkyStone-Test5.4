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
        HOVER
    }
    LiftSystemState state = LiftSystemState.IDLE;
    boolean timedOut = false;


    public LiftSystem(HardwareMap theHardwareMap, Telemetry theTelemetry){
        telemetry = theTelemetry;
        hardwareMap = theHardwareMap;

        grabber = new Grabber(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
    }

    public void initLiftSystem(){
        grabber.initGrabber();
        //here we are moving the grabber to a set position where it is out of the way
        // and convienient
        grabber.closeGrabberNarrow();
        teamUtil.sleep(750);
        grabber.rotate(Grabber.GrabberRotation.INSIDE);
        teamUtil.sleep(750);
        grabber.grabberStow();

        lift.initLift();
        //add limit switch at the top of the lift?
        lift.downPosition(.3, 6000);
        // need to find init positions for the lift base
    }

    public void grabAndStow(String grabberPos, long timeOut){
        state = LiftSystemState.GRAB_AND_STOW;
        teamUtil.log("Grab and Stow");
        long timeOutTime= System.currentTimeMillis()+timeOut;
        timedOut = false;

        if(grabberPos.equals("narrow")){
            grabber.closeGrabberNarrow();
            teamUtil.log("grabbed");

        }else if(grabberPos.equals("wide")){
            grabber.closeGrabberWide();
            teamUtil.log("grabbed");
        }else{
            return;
        }
        // Give the servos enough time to grab the stone
        teamUtil.sleep(500);
        lift.downPositionNoWait(0.3, timeOutTime - System.currentTimeMillis());
        // give a little time for the lift to get far enough down to safely rotate
        teamUtil.sleep(1500);
        if (teamUtil.keepGoing(timeOutTime)){
            grabber.rotate(Grabber.GrabberRotation.INSIDE);
            teamUtil.sleep(500);
            // in case the lift isn't fully down yet...
            while (lift.isBusy()) {
                teamUtil.sleep(100);
            }
        }
        state = LiftSystemState.IDLE;
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Grab and Stow - TIMED OUT!");
        }
        teamUtil.log("Grab and Stow - Finished");
    }

    public void grabAndStowNoWait (final String grabberPos, final long timeOut) {
        if (state == LiftSystemState.IDLE) {
            state = LiftSystemState.GRAB_AND_STOW;
            teamUtil.log("Launching Thread to Grab and Stow");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    grabAndStow(grabberPos, timeOut);
                }
            });
            thread.start();
        }
    }

    public void prepareToGrab(long timeOut){
        state = LiftSystemState.DEPLOY_FOR_PICKUP;
        teamUtil.log("Prepare to Grab");
        long timeOutTime= System.currentTimeMillis()+timeOut;
        timedOut = false;
        grabber.rotate(Grabber.GrabberRotation.INSIDE);
        lift.upPositionNoWait(0.7, timeOut);
        teamUtil.sleep(750);
        if (teamUtil.keepGoing(timeOutTime)) {
            grabber.grabberPickup();
            teamUtil.sleep(500);
        }
        state = LiftSystemState.IDLE;
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Prepare to Grab - TIMED OUT!");
        }
        teamUtil.log("Prepare to Grab - Finished");
    }

    public void prepareToGrabNoWait(final long timeOut) {
        if (state == LiftSystemState.IDLE) {
            state = LiftSystemState.DEPLOY_FOR_PICKUP;
            teamUtil.log("Launching Thread to Prepare to Grab");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    prepareToGrab(timeOut);
                }
            });
            thread.start();
        }
    }

    public void hoverOverFoundation(int level, Grabber.GrabberRotation rotation, long timeOut){
        state = LiftSystemState.HOVER;
        teamUtil.log("Hover");
        long timeOutTime= System.currentTimeMillis()+timeOut;
        timedOut = false;
        grabber.rotate(rotation);
        lift.upPosition(.7, timeOut);
        if (!lift.timedOut && teamUtil.keepGoing(timeOutTime)) {
            lift.goToLevel(level, timeOutTime - System.currentTimeMillis());
        }
        state = LiftSystemState.IDLE;
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Hover - TIMED OUT!");
        }
        teamUtil.log("Hover - Finished");
    }

    public void hoverOverFoundationNoWait(final int level, final Grabber.GrabberRotation rotation, final long timeOut) {
        if (state == LiftSystemState.IDLE) {
            state = LiftSystemState.HOVER;
            teamUtil.log("Launching Thread to Hover");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    hoverOverFoundation(level, rotation, timeOut);
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
