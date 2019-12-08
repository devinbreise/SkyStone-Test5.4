package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class Lift {

    final double LIFT_UP_POWER = .7;
    final double LIFT_DOWN_POWER = .5;

    private final int LIFT_BASE_TOP_LIMIT_ENCODER_CLICKS = 3090;
    private DcMotor liftBase;
    private DcMotor rSpindle;
    private DcMotor lSpindle;
    private RevTouchSensor liftDownLimit;
    //NEVEREST20_ENCODER_CLICKS = 537.6

    // Constants for the spindles
    private final int LEVEL_0 = 430;
    private final int LEVEL_INCREMENT = 570;
    private final double TENSION_POWER = 0.075;

    enum LiftState{
        IDLE,
        MOVING_UP,
        MOVING_DOWN,
    }
    private LiftState liftState = LiftState.IDLE;
    enum ElevatorState{
        IDLE,
        MOVING_UP,
        HOLDING,
        MOVING_DOWN,
    }
    private ElevatorState elevatorState = ElevatorState.IDLE;

    private boolean hasSetZeroSpindle = false;
    private boolean hasSetBaseZero = false;
    boolean timedOut = false;

    HardwareMap hardwareMap;
    Telemetry telemetry;


    public Lift(HardwareMap theHardwareMap, Telemetry theTelmetry){
        teamUtil.log ("Constructing Lift");
        hardwareMap = theHardwareMap;
        telemetry = theTelmetry;
    }

    public void initLift(){
        teamUtil.log ("Initializing Lift");
        liftBase = hardwareMap.dcMotor.get("liftBase");
        liftBase.setDirection(DcMotorSimple.Direction.REVERSE);
        liftBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rSpindle = hardwareMap.dcMotor.get("rSpindle");
        lSpindle = hardwareMap.dcMotor.get("lSpindle");
        lSpindle.setDirection(DcMotorSimple.Direction.REVERSE);
        rSpindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lSpindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftDownLimit = hardwareMap.get(RevTouchSensor.class, "liftDownLimitSwitch");
    }

    public boolean isBusy() {
        return ((liftState != LiftState.IDLE) || (elevatorState != ElevatorState.IDLE));
    }

    public void shutDownLiftBase(){
        liftBase.setPower(0);
    }

    public void downPosition(double power, long timeOut){
        liftState = LiftState.MOVING_DOWN;
        teamUtil.log("Moving Lift Down");
        long timeOutTime= System.currentTimeMillis()+timeOut;
        timedOut = false;

        while(!liftDownLimit.isPressed() && teamUtil.keepGoing(timeOutTime)){
            liftBaseDown();
        }
        shutDownLiftBase();
        liftBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        teamUtil.log("Lift Base encoder reset");
        teamUtil.log("LiftBaseEncoder: " + liftBase.getCurrentPosition());

        hasSetBaseZero = true;
        liftState = LiftState.IDLE;

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Lift Down - TIMED OUT!");
        }
        teamUtil.log("Moving Lift Down - Finished");
    }

    public void downPositionNoWait( final double power, final long timeOut){
        if(liftState == LiftState.IDLE) {
            liftState = LiftState.MOVING_DOWN;
            teamUtil.log("Launching Thread to Move Lift Down");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    downPosition(power, timeOut);
                }
            });
            thread.start();
        }
    }

    // Moves the lift to the up position and returns when finished
    // If this is the first time, also tensions elevator spindles
    // Lift must previously have been moved to the downPosition or this will fail
    public void upPosition(double power, long timeOut){
        liftState = LiftState.MOVING_UP;
        teamUtil.log("Moving Lift Up");
        long timeOutTime= System.currentTimeMillis()+timeOut;
        timedOut = false;

       if (!hasSetBaseZero) {
            teamUtil.log("ERROR: Attempt to Move Lift Up before resetting base");
            liftState = LiftState.IDLE;
            return;
        }
        liftBase.setPower(0);
        liftBase.setTargetPosition(LIFT_BASE_TOP_LIMIT_ENCODER_CLICKS);
        liftBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftBase.setPower(power);
        // wait for the lift to get close to its final position before we move on
        // Using isBusy() on the liftbase motor takes a LONG time due to the PID slowing down at the end
        while ((liftBase.getCurrentPosition()<(.95* LIFT_BASE_TOP_LIMIT_ENCODER_CLICKS)) && teamUtil.keepGoing(timeOutTime)){

        }
        shutDownLiftBase();

/*
        while(liftBase.getCurrentPosition()<3000){
            teamUtil.log("liftEncoder: " + liftBase.getCurrentPosition());
        }
 */
        teamUtil.log("LiftBase is Up" );
        teamUtil.log("LiftBaseEncoder: " + liftBase.getCurrentPosition());
        if(!hasSetZeroSpindle && (timeOutTime > System.currentTimeMillis())){
            tensionLiftString();
            hasSetZeroSpindle = true;
        }
        liftState = LiftState.IDLE;
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Lift Up - TIMED OUT!");
        }
        teamUtil.log("Moving Lift Up - Finished");
    }


    public void upPositionNoWait(final double power, final long timeOut){
        if(liftState == LiftState.IDLE) {
            liftState = LiftState.MOVING_UP;
            teamUtil.log("Launching Thread to Move Lift Up");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    upPosition(power, timeOut);
                }
            });

            thread.start();
        }
    }


    public void liftBaseDown(){
        if(liftDownLimit.isPressed()){
            shutDownLiftBase();
            liftBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            teamUtil.log("Lift Base encoder reset");

        } else {
            liftBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftBase.setPower(-liftBasePower);
        }
    }


    public double getBasePosition(){
        return liftBase.getCurrentPosition();
    }



    // we need to figure out where this sort of code should be run as the lift is
    // put through various operations
    public void tensionLiftString(){
        rSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSpindle.setPower(TENSION_POWER);
        lSpindle.setPower(TENSION_POWER);
        teamUtil.sleep(2000); // for testing, this is not OK in an op mode
        rSpindle.setPower(0);
        lSpindle.setPower(0);
        rSpindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void tensionLiftStringContinuous(){
        rSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSpindle.setPower(TENSION_POWER);
        lSpindle.setPower(TENSION_POWER);
    }

    public void liftLoop(Lift lift){
        if(rSpindle.getCurrentPosition()<0 || lSpindle.getCurrentPosition()<0){
            lift.tensionLiftStringContinuous();
        }
    }

    // Move the elevator up to the specified level using FTC PID controller
    public void goToLevel(int level, long timeOut) {
        elevatorState = ElevatorState.MOVING_UP;
        teamUtil.log("Going to level: " + level);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        teamUtil.log("firstSpindlePosition: " + rSpindle.getCurrentPosition());
        rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSpindle.setTargetPosition(LEVEL_0 + (LEVEL_INCREMENT * level));
        lSpindle.setTargetPosition(LEVEL_0 + (LEVEL_INCREMENT * level));
        rSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rSpindle.setPower(.99);
        lSpindle.setPower(.99);
        while (rSpindle.isBusy() && teamUtil.keepGoing(timeOutTime)) {

        }
        elevatorState = ElevatorState.HOLDING;
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Go To Level - TIMED OUT!");
            goToBottomNoWait();
        } else {
            teamUtil.log("Go To Level - Finished");
        }
    }

    public void goToLevelNoWait(final int level, final long timeOut){
        if (elevatorState == ElevatorState.IDLE) {
            elevatorState = ElevatorState.MOVING_UP;
            teamUtil.log("Launching Thread to Go To Level");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    goToLevel(level, timeOut);
                }
            });
            thread.start();
        }
    }
    // Drop the lift to its lowest point
    // TODO : COACH - This needs to be an active mechanism (i.e. run the spindles to let
    // out string at an appropriate pace (for extra credit, make it faster for the higher levels
    // and slow down as it approaches the bottom so it doesn't slam
    public void goToBottom() {
        teamUtil.log("Moving Elevator to Bottom");
        elevatorState = ElevatorState.MOVING_DOWN;
        rSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSpindle.setPower(-0.1);
        lSpindle.setPower(-0.1);
        teamUtil.sleep(2500);
        tensionLiftString();
        elevatorState = ElevatorState.IDLE;
        teamUtil.log("Moving Elevator to Bottom - Finished");
    }

    public void goToBottomNoWait(){
        teamUtil.log("elevatorstate is " + elevatorState);
        if ((elevatorState == ElevatorState.HOLDING)){
            elevatorState = ElevatorState.MOVING_UP;
            teamUtil.log("Launching Thread to Move Elevator to Bottom");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    goToBottom();
                }
            });
            thread.start();
        }
    }

    public void liftTelemetry(){
        telemetry.addData("Lift Motor Power:", liftBasePower);
        telemetry.addData("Base:", liftBase.getCurrentPosition());
        telemetry.addData("liftDownLimit:", liftDownLimit.isPressed());
        telemetry.addData("lSpindle:", lSpindle.getCurrentPosition());
        telemetry.addData("rSpindle:", rSpindle.getCurrentPosition());
    }


    /////////////////////////////////////////////////////////////////////////
    // Some stuff for testing only

    // Keep track of when controls are updated to limit how fast values can change
    private long nextControlUpdate = System.currentTimeMillis();
    private final long CONTROL_INTERVAL = 500;
    private double liftBasePower = .5;


    public void increaseLiftBasePower(){
        // If enough time has passed since we last updated the power
        if (System.currentTimeMillis() > nextControlUpdate)
        {
            liftBasePower = liftBasePower + 0.1;
            nextControlUpdate = System.currentTimeMillis() + CONTROL_INTERVAL;
        }
    }
    public void decreaseLiftBasePower(){
        // If enough time has passed since we last updated the power
        if (System.currentTimeMillis() > nextControlUpdate)
        {
            liftBasePower = liftBasePower - 0.1;
            nextControlUpdate = System.currentTimeMillis() + CONTROL_INTERVAL;
        }
    }

    public void liftBaseUp(){
        if(liftBase.getCurrentPosition()< LIFT_BASE_TOP_LIMIT_ENCODER_CLICKS){
            liftBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftBase.setPower(liftBasePower);
        }
    }

}



