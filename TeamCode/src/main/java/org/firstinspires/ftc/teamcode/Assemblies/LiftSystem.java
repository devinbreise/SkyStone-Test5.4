package org.firstinspires.ftc.teamcode.Assemblies;



import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class LiftSystem {
    public Grabber grabber;
    public Lift lift;
    Telemetry telemetry;
    HardwareMap hardwareMap;
    public enum LiftSystemState{
        IDLE,
        GRAB, // MOVING TO GRAB
        GRAB_AND_STOW, // MOVING TO GRAB AND STOW
        LIFT_STOW, // moving to put lift system away without grab
        PREPARE_TO_GRAB, // MOVING TO PREPARE TO GRAB
        MOVING_TO_HOVER, // moving to a new hover position
        HOVER // ELEVATOR HOVERING (not moving)
    }
    public LiftSystemState state = LiftSystemState.IDLE;
    public boolean preparedToGrab = false; // TODO: This is a bit of a hack, this bit of state should be incorporated into the overall state enum for this class
    boolean timedOut = false;
    //boolean isStowed = false;


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public LiftSystem(HardwareMap theHardwareMap, Telemetry theTelemetry){
        telemetry = theTelemetry;
        hardwareMap = theHardwareMap;

        grabber = new Grabber(hardwareMap, telemetry);
        lift = new Lift(hardwareMap, telemetry);
    }

    public boolean isMoving() {
        return (state != LiftSystemState.IDLE) && (state != LiftSystemState.HOVER);
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // move the lift system down and into stowed position while calibrating
    public void initLiftSystem(){
        grabber.initGrabber();
        lift.initLift();
        if (!lift.liftBaseIsDown()) {
            //move the grabber to a set position where it is out of the way for the lift base to go down
            grabber.grabberRotatePos();
            teamUtil.sleep(750);
        }
        // move the lift down
        lift.moveLiftBaseDown(.3, 8000);
        lift.calibrateSpindles();
        if (grabber.rotation != Grabber.GrabberRotation.INSIDE) {  // rotate to inside position

            // WARNING!  if the lift base was down and the paddles are in a position that can't rotate
            // we will crash here when we try to rotate
            // but the alternative is to always extend the paddles to a safe rotation position, which will extend past
            // the front of the robot and possibly crash into the wall...
            grabber.rotate(Grabber.GrabberRotation.INSIDE);
            teamUtil.sleep(750);
        }
        grabber.grabberStow();
        state = LiftSystemState.IDLE;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void prepareToGrab(long timeOut){
        state = LiftSystemState.PREPARE_TO_GRAB;
        teamUtil.log("Prepare to grab");
        long timeOutTime= System.currentTimeMillis()+timeOut;
        timedOut = false;

        // Start the lift moving up if needed
        if (!lift.liftBaseIsUp()) {
            lift.moveLiftBaseUpNoWait(0.95, timeOut);
        }

        // meanwhile see about getting the grabber in the right position
        //teamUtil.log("Grabber Rotation: " + grabber.rotation);
        if (grabber.rotation != Grabber.GrabberRotation.INSIDE) { // need to rotate inside
            teamUtil.log("Rotating Grabber Inside");
            if (!grabber.isSafeToRotate()) { // but don't try to rotate if the paddles are in a bad position
                grabber.closeGrabberWide();
                teamUtil.log("Moving paddles to safe position to rotate");
                teamUtil.sleep(500);
            }
            if (!lift.isSafeToRotate()) {// make sure the lift is in safe position to rotate (we can't rotate in the up position at the bottom of the elevator)
                while (!lift.isSafeToElevate() && teamUtil.keepGoing(timeOutTime)) {
                    teamUtil.sleep(100); // wait until the lift is far enough up to elevate
                }
                if (teamUtil.keepGoing(timeOutTime)) {
                    lift.moveElevator(lift.SAFE_TO_ROTATE,2000 );
                }
            }
            grabber.rotate(Grabber.GrabberRotation.INSIDE);
            teamUtil.sleep(500);
        }
        grabber.openGrabber();
        // start the elevator when we can
        while (!lift.isSafeToElevate() && teamUtil.keepGoing(timeOutTime)) {
            teamUtil.sleep(100);
        }
        // hover at foundation level ... we will dip down to grab the stone
        teamUtil.log("Moving up to HOVER_FOR_GRAB"); // TODO use moveElevatorToPosition() with a lower position but still above bottom
        lift.moveElevator(lift.HOVER_FOR_GRAB, 2000); // a little closer to the ground then level 0
        //lift.moveElevatorToLevelNoWait(0, 3000);
        state = LiftSystemState.IDLE;
        preparedToGrab = true;
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Prepare to grab - TIMED OUT!");
        }
        teamUtil.log("Prepare to grab - Finished");
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void prepareToGrabNoWait(final long timeOut) {
        if (!isMoving()) {
            state = LiftSystemState.PREPARE_TO_GRAB;
            teamUtil.log("Launching Thread to Prepare to grab");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    prepareToGrab(timeOut);
                }
            });
            thread.start();
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // grab a stone and stow the lift for travel
    public void grabAndStow(long timeOut){
//        if (!lift.liftBaseIsUp()) {
//            teamUtil.log("WARNING: grabAndStow called when lift was not up");
//            state = LiftSystemState.IDLE;
//            return;
//        }
        state = LiftSystemState.GRAB_AND_STOW;
        preparedToGrab = false;
        teamUtil.log("grab and Stow");
        long timeOutTime= System.currentTimeMillis()+timeOut;
        timedOut = false;
        lift.moveElevatorToBottom();  // dip down for the grab

        grabber.closeGrabberWide();

        // Give the servos enough time to grab the stone
        teamUtil.sleep(750);
        lift.moveLiftBaseDownNoWait(0.5, timeOutTime - System.currentTimeMillis());
        // give a little time for the lift to get far enough down to safely rotate
        teamUtil.sleep(1000);
        if (teamUtil.keepGoing(timeOutTime)){
            grabber.rotate(Grabber.GrabberRotation.INSIDE);
            // in case the lift isn't fully down yet...
            while (lift.isBusy()) {
                teamUtil.sleep(100);
            }
        }
        state = LiftSystemState.IDLE;
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("grab and Stow - TIMED OUT!");
        }
        teamUtil.log("grab and Stow - Finished");
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void grabAndStowNoWait (final long timeOut) {
        if (!isMoving()) {
            state = LiftSystemState.GRAB_AND_STOW;
            teamUtil.log("Launching Thread to grab and Stow");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    grabAndStow(timeOut);
                }
            });
            thread.start();
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // dip down, grab a stone and then return to level 0
    public void grab(long timeOut){
        state = LiftSystemState.GRAB;
        preparedToGrab = false;
        teamUtil.log("grab");
        if (!lift.liftBaseIsUp()) {
            teamUtil.log("WARNING: grab called when lift was not up");
            state = LiftSystemState.IDLE;
            return;
        }
        long timeOutTime= System.currentTimeMillis()+timeOut;
        timedOut = false;
        lift.moveElevatorToBottom();  // dip down for the grab

        grabber.closeGrabberWide();

        // Give the servos enough time to grab the stone
        teamUtil.sleep(500);
        lift.moveElevator(lift.HOVER_FOR_GRAB, timeOut); // a little closer to the ground then level 0
        //lift.moveElevatorToLevel(0, timeOut);
        teamUtil.log("grab - Finished");
        state = LiftSystemState.IDLE;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void grabNoWait(final long timeOut) {
        if (!isMoving()) {
            state = LiftSystemState.GRAB;
            teamUtil.log("Launching Thread to Grab");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    grab(timeOut);
                }
            });
            thread.start();
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // dip down, align a stone, drop capstone on it, grab it, and then return to level 0
    public void capAndGrab(long timeOut){
        state = LiftSystemState.GRAB;
        preparedToGrab = false;
        teamUtil.log("cap & grab");
        if (!lift.liftBaseIsUp()) {
            teamUtil.log("WARNING: cap & grab called when lift was not up");
            state = LiftSystemState.IDLE;
            return;
        }
        long timeOutTime= System.currentTimeMillis()+timeOut;
        timedOut = false;
        lift.moveElevatorToBottom();  // dip down for the grab

        // Give the servos enough time to grab the stone
        grabber.closeGrabberWide();
        teamUtil.sleep(500);
        grabber.dropCapstone();
        // Give the servos enough time to cap
        teamUtil.sleep(1250);
        grabber.closeGrabberWide();
        // Give the servos enough time to grab the stone
        teamUtil.sleep(1250);
        lift.moveElevator(lift.HOVER_FOR_GRAB, timeOut); // a little closer to the ground then level 0
        //lift.moveElevatorToLevel(0, timeOut);
        teamUtil.log("cap & grab - Finished");
        state = LiftSystemState.IDLE;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void capAndGrabNoWait(final long timeOut) {
        if (!isMoving()) {
            state = LiftSystemState.GRAB;
            teamUtil.log("Launching Thread to Cap and Grab");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    capAndGrab(timeOut);
                }
            });
            thread.start();
        }
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // get ready to drop a stone (presumbably being held when this is called) at the specified level
    // in the specified rotation.
    // This will raise the lift if needed and rotate the grabber
    public void hoverOverFoundation(int level, Grabber.GrabberRotation rotation, long timeOut) {
        state = LiftSystemState.MOVING_TO_HOVER;
        preparedToGrab = false;
        teamUtil.log("Moving to Hover");
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        if (!lift.liftBaseIsUp()) { // raise lift base if needed
            lift.moveLiftBaseUpNoWait(.7, timeOut);
        }
        if (grabber.rotation != rotation) { // we need to rotate the grabber
            if (!grabber.isSafeToRotate()) { // make sure the paddles are in a safe position to rotate
                grabber.closeGrabberWide();
                teamUtil.log("WARNING: hoverOverFoundation called while paddles not in a grabbing position");
                teamUtil.sleep(500);
            }
            if (!lift.isSafeToRotate()) {// make sure the lift is in safe position to rotate (we can't rotate in the up position at the bottom of the elevator)
                while (!lift.isSafeToElevate() && teamUtil.keepGoing(timeOutTime)) {
                    teamUtil.sleep(100); // wait until the lift is far enough up to elevate
                }
                if (teamUtil.keepGoing(timeOutTime)) {
                    lift.moveElevator(lift.SAFE_TO_ROTATE,2000 );
                }
            }
            // rotate now that we have made sure it is safe
            grabber.rotate(rotation);
        }
        // wait for the lift to be far enough up
        while (!lift.isSafeToElevate() && teamUtil.keepGoing(timeOutTime )) {
            // waiting to elevate
        }
        if (!lift.timedOut && teamUtil.keepGoing(timeOutTime)) {
            lift.moveElevatorToLevel(level, 3000);
        }
        state = LiftSystemState.HOVER;
        timedOut = (System.currentTimeMillis() > timeOutTime) || lift.timedOut;
        if (timedOut) {
            teamUtil.log("Hover - TIMED OUT!");
        }
        teamUtil.log("Hover - Finished");
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void hoverOverFoundationNoWait(final int level, final Grabber.GrabberRotation rotation, final long timeOut) {
        if (!isMoving()) {
            state = LiftSystemState.MOVING_TO_HOVER;
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

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Drop the lift system without grabbing anything.
    public void putAwayLiftSystem ( long timeOut) {

        state = LiftSystemState.LIFT_STOW;
        // Make sure liftbase is up
        if (!lift.liftBaseIsUp()) {
            teamUtil.log("WARNING: putAwayLiftSystem called when lift was not up");
            state = LiftSystemState.IDLE;
            return;
        }
        // See if we need to rotate the servos
        if (grabber.rotation != Grabber.GrabberRotation.INSIDE) {
            // we need to rotate the servos so first we need to make sure our paddles our in a position where it is safe to rotate
            if (!grabber.isSafeToRotate()) {
                // we need to adjust the paddle positions to rotate
                grabber.grabberRotatePos();
                teamUtil.sleep(1250); // worst case, stowed or capstone release position
            }
            grabber.rotate(Grabber.GrabberRotation.INSIDE);
            teamUtil.sleep(750); // wait for rotation to complete
        }
        // put the paddles in a position that is appropriate for stowed if they are not already
        if (!grabber.isSafeToRotate()) {
            grabber.grabberRotatePos();
        }
        lift.moveElevatorToBottom();
        lift.moveLiftBaseDown(.95, timeOut);
        state=LiftSystemState.IDLE;
        preparedToGrab = false;
    }

    public void putAwayLiftSystemNoWait(final long timeOut) {
        if (!isMoving()) {
            state = LiftSystemState.LIFT_STOW;
            teamUtil.log("Launching Thread to Hover");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    putAwayLiftSystem(timeOut);
                }
            });
            thread.start();
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void drop(){
        grabber.grabberPickup();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void elevatorDown(){
        lift.moveElevatorToBottomNoWait();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void openGrabber(){
        grabber.openGrabber();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void closeGrabberWide(){
        grabber.closeGrabberWide();
    }
}
