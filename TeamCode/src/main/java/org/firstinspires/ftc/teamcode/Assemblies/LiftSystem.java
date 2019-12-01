package org.firstinspires.ftc.teamcode.Assemblies;



import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Assemblies.Grabber;
import org.firstinspires.ftc.teamcode.Assemblies.Lift;
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

        lift.initLift();
        //add limit switch at the top of the lift
        lift.downPosition(.3);
        // need to find init positions for the lift base
    }

    public void grabAndStow(){
        state = LiftSystemState.GRAB_AND_STOW;
        grabber.closeGrabberWide();
        teamUtil.log("grabbed");
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

    public void grabAndStowNoWait () {
        if (state == LiftSystemState.IDLE) {
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    grabAndStow();
                }
            });
            thread.start();
        }
    }

    public void deployForPickUp(){
        state = LiftSystemState.DEPLOY_FOR_PICKUP;
        grabber.rotateInside();
        lift.upPositionNoWait(0.7);
        teamUtil.sleep(750);
        grabber.grabberPickup();
        teamUtil.sleep(500);
        state = LiftSystemState.IDLE;
    }

    public void deployForPickUpNoWait () {
        if (state == LiftSystemState.IDLE) {
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    deployForPickUp();
                }
            });
            thread.start();
        }
    }

    public void deployForPlace(int level, Grabber.GrabberRotation rotation){
        grabber.rotate(rotation);
        lift.upPosition(.7);
        lift.goToLevel(level);
    }

    public void deployForPlaceNoWait (final int level, final Grabber.GrabberRotation rotation) {
        if (state == LiftSystemState.IDLE) {
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    deployForPlace(level, rotation);
                }
            });
            thread.start();
        }
    }
}
