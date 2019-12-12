package org.firstinspires.ftc.teamcode.Assemblies;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class LiftSystem {
    Grabber grabber;
    public Lift lift;
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
    boolean isStowed = false;


    public LiftSystem(HardwareMap theHardwareMap, Telemetry theTelemetry){
        telemetry = theTelemetry;
        hardwareMap = theHardwareMap;

        grabber = new Grabber(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
    }

    public void initLiftSystem(){
        isStowed = false;
        grabber.initGrabber();
        lift.initLift();
        if (!lift.liftBaseIsDown()) {
            //move the grabber to a set position where it is out of the way for the lift base to go down
            grabber.closeGrabberNarrow();
            teamUtil.sleep(750);
        }

        //add limit switch at the top of the lift?
        // move the lift down and rest the encoder
        lift.downPosition(.3, 8000);
        lift.tensionLiftStringContinuous();
        grabber.rotate(Grabber.GrabberRotation.INSIDE);
        teamUtil.sleep(750);
         //MAYBE
        grabber.grabberStow();

        // need to find init positions for the lift base
    }

    public void resetSpindles(){
        lift.rSpindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.lSpindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            teamUtil.log("BAD INPUT to Grab and Stow");
            state = LiftSystemState.IDLE;
            return;
        }
        // Give the servos enough time to grab the stone
        teamUtil.sleep(500);
        lift.downPositionNoWait(0.5, timeOutTime - System.currentTimeMillis());
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
        isStowed = true;
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

    public void grabAndDip(){
        grabber.openGrabber();
        lift.goToBottom();
        teamUtil.sleep(1250);
        teamUtil.log("going to bottom ;-;");
        grabber.closeGrabberWide();
        teamUtil.sleep(750);
        lift.goToLevel(0, 5000);
        teamUtil.log("grabbed and gone back up!");
    }

    public void prepareToGrab(long timeOut){
        isStowed = false;
        state = LiftSystemState.DEPLOY_FOR_PICKUP;
        teamUtil.log("Prepare to Grab");
        long timeOutTime= System.currentTimeMillis()+timeOut;
        timedOut = false;
        grabber.rotate(Grabber.GrabberRotation.INSIDE);
        lift.upPositionNoWait(0.7, timeOut);
        // let the lift get moving first
        teamUtil.sleep(750);
        if (teamUtil.keepGoing(timeOutTime)) {
            // then move the grabber panels
            grabber.grabberPickup();
            teamUtil.sleep(500);
        }
        // in case the lift isn't fully up yet...
        while (lift.isBusy()) {
            // This is where we can elevate once lift.safeToElevate() returns true...
            teamUtil.sleep(100);
        }
        lift.goToLevelNoWait(0, 5000);
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

    public void hoverOverFoundation(int level, Grabber.GrabberRotation rotation, long timeOut) {
        if (lift.isSafeToElevate()) {
            state = LiftSystemState.HOVER;
            teamUtil.log("Hover");
            long timeOutTime = System.currentTimeMillis() + timeOut;
            timedOut = false;
            if (isStowed) {
                grabber.rotate(rotation);
                // This is where we can use the "noWait" version of upPosition and then, in the loop while we wait for it to finish
                // we can elevate once lift.safeToElevate() returns true...
                lift.upPosition(.7, timeOut);
                if (!lift.timedOut && teamUtil.keepGoing(timeOutTime)) {
                    lift.goToLevel(level, timeOutTime - System.currentTimeMillis());
                }

            } else {
                grabber.closeGrabberWide();
                lift.upPosition(.7, timeOut);
                if (!lift.timedOut && teamUtil.keepGoing(timeOutTime)) {
                    lift.goToLevel(level, timeOutTime - System.currentTimeMillis());
                    grabber.rotate(rotation);
                }
            }
            state = LiftSystemState.IDLE;
            timedOut = (System.currentTimeMillis() > timeOutTime);
            if (timedOut) {
                teamUtil.log("Hover - TIMED OUT!");
            }
            teamUtil.log("Hover - Finished");
        }
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
