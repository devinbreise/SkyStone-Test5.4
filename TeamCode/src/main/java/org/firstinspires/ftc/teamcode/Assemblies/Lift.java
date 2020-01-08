package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class Lift {

    HardwareMap hardwareMap;
    Telemetry telemetry;

    private DcMotor liftBase;
    public DcMotor rSpindle;
    public DcMotor lSpindle;
    private RevTouchSensor liftDownLimit;
    private DigitalChannel liftUpMagSwitch1, liftUpMagSwitch2;

    // Constants for the base lift (when using the encoder)
    private final int LIFT_BASE_TOP_LIMIT_ENCODER_CLICKS = 2725;
    private final int LIFT_BASE_SAFE_TO_ELEVATE = 2000;

    // Constants for the elevator spindles
    public final int BOTTOM = 0;
    private final int TOP = 5970;
    public final int HOVER_FOR_GRAB = 0; // this is about 1" up from bottom
    private final int LEVEL_0 = 370; //was 430
    private final int LEVEL_INCREMENT = 560;
    public final int SAFE_TO_ROTATE = LEVEL_0 + LEVEL_INCREMENT;

    private final double TENSION_POWER = 0.098;
    private final int MAX_LEVELS = 10;


    enum LiftState {
        IDLE,
        MOVING_UP,
        MOVING_DOWN,
    }

    enum ElevatorState {
        IDLE, // bottom
        MOVING_UP,
        HOLDING,
        MOVING_DOWN,
        MANUAL_UP,
        MANUAL_DOWN
    }

    private LiftState liftState = LiftState.IDLE;
    private ElevatorState elevatorState = ElevatorState.IDLE;

    private boolean spindlesCalibrated = false;
    private boolean baseCalibrated = false;
    private boolean safeToElevate = false;
    boolean timedOut = false;


    public Lift(HardwareMap theHardwareMap, Telemetry theTelmetry) {
        teamUtil.log("Constructing Lift");
        hardwareMap = theHardwareMap;
        telemetry = theTelmetry;
    }

    public void initLift() {
        teamUtil.log("Initializing Lift");
        spindlesCalibrated = false;
        baseCalibrated = false;
        safeToElevate = false;
        timedOut = false;
        liftState = LiftState.IDLE;
        elevatorState = ElevatorState.IDLE;

        liftBase = hardwareMap.dcMotor.get("liftBase");
        liftBase.setDirection(DcMotorSimple.Direction.REVERSE);
        liftBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rSpindle = hardwareMap.dcMotor.get("rSpindle");
        lSpindle = hardwareMap.dcMotor.get("lSpindle");
        rSpindle.setDirection(DcMotorSimple.Direction.REVERSE);
        rSpindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lSpindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftDownLimit = hardwareMap.get(RevTouchSensor.class, "liftDownLimitSwitch");
        liftUpMagSwitch1 = hardwareMap.get(DigitalChannel.class, "liftUpMagSwitch1");
        liftUpMagSwitch2 = hardwareMap.get(DigitalChannel.class, "liftUpMagSwitch2");
        liftUpMagSwitch1.setMode(DigitalChannel.Mode.INPUT);
        liftUpMagSwitch2.setMode(DigitalChannel.Mode.INPUT);
    }

    // call after initialization to calibrate the base lift and spindles.
    // This may move the liftbase to the bottom
    public void calibrate() {
        //TODO
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // true if things are moving or the elevator is holding a position
    public boolean isBusy() {
        return ((liftState != LiftState.IDLE) || ((elevatorState != ElevatorState.IDLE) && (elevatorState != ElevatorState.HOLDING)));
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public boolean liftBaseIsDown() {
        return liftDownLimit.isPressed();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public boolean liftBaseIsUp() {
//        return (!liftUpMagSwitch1.getState()) || (!liftUpMagSwitch2.getState());
        return (liftBase.getCurrentPosition() > (.95 * LIFT_BASE_TOP_LIMIT_ENCODER_CLICKS));

    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public boolean isSafeToElevate() {
        return safeToElevate || (baseCalibrated && spindlesCalibrated && liftBaseIsUp());
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // true if the lift is in a position that supports rotation...the paddle positions might still be a problem.
    public boolean isSafeToRotate() {
        if (liftBaseIsDown()) {
            return true;
        } else if (liftBaseIsUp()) {
            return elevatorState == ElevatorState.HOLDING && lSpindle.getCurrentPosition() >= SAFE_TO_ROTATE * .95;
        } else { // lift is part way up
            return !safeToElevate; // OK to rotate if we are in the bottom part of the lift base range of motion
        }
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // BASE LIFT OPERATIONS
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void shutDownLiftBase() {
        liftBase.setPower(0);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void liftBaseDown() {
        if (liftDownLimit.isPressed()) {
            shutDownLiftBase();
            liftBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            safeToElevate = false;
            teamUtil.log("Lift Base encoder reset");

        } else {
            liftBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftBase.setPower(-liftBasePower);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public double getBasePosition() {
        return liftBase.getCurrentPosition();
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void moveBaseDownOriginal(double power, long timeOut) {
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        teamUtil.log("Limit Switch:" + liftDownLimit.isPressed());

        while (!liftDownLimit.isPressed() && teamUtil.keepGoing(timeOutTime)) {
            teamUtil.log("Lift Base Encoder:" + liftBase.getCurrentPosition());
            liftBaseDown();
        }
        shutDownLiftBase();
        liftBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        teamUtil.log("Lift Base encoder reset");
        teamUtil.log("LiftBaseEncoder: " + liftBase.getCurrentPosition());

        baseCalibrated = true;
        liftState = LiftState.IDLE;

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Lift Down - TIMED OUT!");
        }
        teamUtil.log("Moving Lift Down - Finished");

    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void moveBaseDownFaster(double power, long timeOut) {
        if (!liftBaseIsUp()) { // should not be called if we are not at the top
            teamUtil.log("ERROR: moveBaseDownFaster called when base not at top");
            liftState = LiftState.IDLE;
            return;
        }
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        teamUtil.log("Limit Switch:" + liftDownLimit.isPressed());
        liftBasePower = .75;
        liftBaseDown();
        teamUtil.sleep(1000);
        liftBasePower = .4;
        liftBaseDown();
        while (!liftDownLimit.isPressed() && teamUtil.keepGoing(timeOutTime)) {
            //teamUtil.log("Lift Base Encoder:"+liftBase.getCurrentPosition());
            liftBaseDown();
        }
        shutDownLiftBase();
        liftBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        teamUtil.log("Lift Base encoder reset");
        teamUtil.log("LiftBaseEncoder: " + liftBase.getCurrentPosition());

        baseCalibrated = true;
        liftState = LiftState.IDLE;

        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Lift Down - TIMED OUT!");
        }
        teamUtil.log("Moving Lift Down - Finished");
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveLiftBaseDown(double power, long timeOut) {
        if (liftBaseIsDown()) { // already down, do nothing...
            teamUtil.log("WARNING: moveLiftBaseDown called when lift was already down");
            liftState = LiftState.IDLE;
            baseCalibrated = true;
            safeToElevate = false;
            liftBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return;
        }
        if (elevatorState != ElevatorState.IDLE) { // elevator must be down
            teamUtil.log("ERROR: moveLiftBaseDown called when elevator not idle");
            return;
        }

        liftState = LiftState.MOVING_DOWN;
        safeToElevate = false;
        teamUtil.log("Moving Lift Down");

        if (liftBaseIsUp()) { // if we are at the top, lets go a little faster
            moveBaseDownFaster(power, timeOut);
        } else {
            moveBaseDownOriginal(power, timeOut);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveLiftBaseDownNoWait(final double power, final long timeOut) {
        if (liftState == LiftState.IDLE) {
            liftState = LiftState.MOVING_DOWN;
            teamUtil.log("Launching Thread to Move Lift Down");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    moveLiftBaseDown(power, timeOut);
                }
            });
            thread.start();
        }
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Lift base must previously have been calibrated (moved to the downPosition) or this will fail
    private void moveBaseUpUsingEncoders(double power, long timeOut) {
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        if (!baseCalibrated) {
            teamUtil.log("ERROR: Attempt to Move Lift Up before resetting base");
            liftState = LiftState.IDLE;
            return;
        }
        int mockTarget = (int) (LIFT_BASE_TOP_LIMIT_ENCODER_CLICKS * 1.05);
        liftBase.setPower(0);
        liftBase.setTargetPosition(mockTarget);
        liftBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftBase.setPower(power);
        // wait for the lift to get close to its final position before we move on
        // Using isBusy() on the liftbase motor takes a LONG time due to the PID slowing down at the end
        while ((liftBase.getCurrentPosition() < (LIFT_BASE_TOP_LIMIT_ENCODER_CLICKS)) && teamUtil.keepGoing(timeOutTime)) {
            teamUtil.log("liftEncoder: " + liftBase.getCurrentPosition());
            if (liftBase.getCurrentPosition() > LIFT_BASE_SAFE_TO_ELEVATE) {
                safeToElevate = true;
            }
        }
        shutDownLiftBase();
        teamUtil.log("LiftBase is Up");
        teamUtil.log("LiftBaseEncoder: " + liftBase.getCurrentPosition());
        liftState = LiftState.IDLE;
        safeToElevate = true;
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Lift Up - TIMED OUT!");
        }
        teamUtil.log("Moving Lift Up - Finished");
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void moveBaseUpUsingLimits(double power, long timeOut) {
        // This is a hack until we get the base lift encoder working again
        // It ignores the passed in power since the power and timing settings in here
        // are dependent on each other.
        teamUtil.log("Moving Lift Up Using Limit Switches");
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        liftBase.setPower(0);
        liftBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftBase.setPower(.95); // go up fast
        teamUtil.theOpMode.sleep(1000); // wait for the lift to get close to its final position
        safeToElevate = true; // far enough up now to operate elevator
        teamUtil.log("Safe to operate elevator");
        liftBase.setPower(.5); // slow down as we get near the top
        while (!liftBaseIsUp()) {
            // wait till the limit switch is triggered
            //teamUtil.log("waiting for mag limits" );
        }
        // let it go just a bit further to get to vertical
        //teamUtil.sleep(300);
        liftBase.setPower(0); // stop

        teamUtil.log("LiftBase is Up");
        teamUtil.log("LiftBaseEncoder: " + liftBase.getCurrentPosition());
        liftState = LiftState.IDLE;
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Lift Up - TIMED OUT!");
        }
        teamUtil.log("Moving Lift Up - Finished");
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Moves the lift to the up position and returns when finished
    public void moveLiftBaseUp(double power, long timeOut) {
        if (liftBaseIsUp()) { // already up, do nothing...
            teamUtil.log("WARNING: moveLiftBaseUp called when lift was already Up");
            liftState = LiftState.IDLE;
            return;
        }
        liftState = LiftState.MOVING_UP;
        teamUtil.log("Moving Lift Up");
        safeToElevate = false;

        // OR moveBaseUpUsingLimits(power, timeOut);
        moveBaseUpUsingEncoders(power, timeOut);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveLiftBaseUpNoWait(final double power, final long timeOut) {
        if (liftState == LiftState.IDLE) {
            liftState = LiftState.MOVING_UP;
            teamUtil.log("Launching Thread to Move Lift Up");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    moveLiftBaseUp(power, timeOut);
                }
            });

            thread.start();
        }

    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // ELEVATOR OPERATIONS
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void forceTensionLiftString(long msecs) {
        rSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSpindle.setPower(TENSION_POWER);
        lSpindle.setPower(TENSION_POWER);
        teamUtil.sleep(msecs); // for testing, this is not OK in an op mode
        rSpindle.setPower(0);
        lSpindle.setPower(0);
        rSpindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void tensionLiftStringContinuous() {
        rSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSpindle.setPower(TENSION_POWER);
        lSpindle.setPower(TENSION_POWER);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // call this method during init while the liftbase is down to calibrate the spindles
    public void calibrateSpindles() {
        teamUtil.log("Calibrating Spindles");
        if (!liftBaseIsDown()) {
            teamUtil.log("ERROR: calibrateSpindles called while lift base wasn't down");
            return;
        }
        rSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSpindle.setPower(TENSION_POWER);
        lSpindle.setPower(TENSION_POWER);
        teamUtil.sleep(500); // let them get going
        do {
            long lastLeft = lSpindle.getCurrentPosition();
            long lastRight = rSpindle.getCurrentPosition();
            teamUtil.sleep(250);
            // if things aren't moving, we have stalled
            if ((lSpindle.getCurrentPosition() == lastLeft) && (rSpindle.getCurrentPosition() == lastRight)) {
                rSpindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lSpindle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rSpindle.setPower(0);
                lSpindle.setPower(0);
                rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                spindlesCalibrated = true;
                teamUtil.log("Calibrating Spindles - Finished");

                return;
            }
            teamUtil.log("Spindle Encoders:" + lastLeft + " " + lastRight);

        } while (true);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Stop the elevator wherever it is and hold
    public void manualHoldElevator() {
        // if we are not moving, nothing to do
        if (elevatorState != ElevatorState.MANUAL_UP && elevatorState != ElevatorState.MANUAL_DOWN) {
            return;
        }
        rSpindle.setTargetPosition(rSpindle.getCurrentPosition());
        lSpindle.setTargetPosition(lSpindle.getCurrentPosition());
        rSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rSpindle.setPower(.99);
        lSpindle.setPower(.99);
        elevatorState = ElevatorState.HOLDING;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Move up to a specific position as fast as possible
    // This will only work if you are moving the elevator up...
    private void moveElevatorUp(int position, long timeOut) {
        if (!isSafeToElevate()) {
            teamUtil.log("ERROR: called moveElevatorUp while not safe to elevate");
            return;
        }
        if (lSpindle.getCurrentPosition() > position || rSpindle.getCurrentPosition() > position) {
            teamUtil.log("ERROR: called moveElevatorUp with a target position below current position");
            return;
        }
        elevatorState = ElevatorState.MOVING_UP;
        teamUtil.log("Moving Elevator UP to: " + position);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;
        int mockTarget = position + 200;
        teamUtil.log("firstSpindlePosition: " + rSpindle.getCurrentPosition());
        rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSpindle.setTargetPosition(mockTarget);
        lSpindle.setTargetPosition(mockTarget);
        rSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rSpindle.setPower(.99);
        lSpindle.setPower(.99);
        while (lSpindle.getCurrentPosition() < position && rSpindle.getCurrentPosition() < position && teamUtil.keepGoing(timeOutTime)) {
            //teamUtil.log("Spindles: " + lSpindle.getCurrentPosition()+" : "+rSpindle.getCurrentPosition()); // uncomment for play by play...
        }
        // set the new target for the FTC code to the actual target to hold there
        lSpindle.setTargetPosition(position);
        rSpindle.setTargetPosition(position);
        elevatorState = ElevatorState.HOLDING;
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Elevator UP - TIMED OUT!");
            moveElevatorToBottomNoWait();
        } else {
            teamUtil.log("Moving Elevator UP - Finished");
        }

    }

    // start the elevator moving up slowly, presumably under manual control
    // This expects to be called over and over while a control is being pressed so
    // that we can stop if we are too high!
    public void moveElevatorUpSlowly() {
        if (!isSafeToElevate()) {
            teamUtil.log("ERROR: called moveElevatorUpSlowly while not safe to elevate");
            return;
        }
        if (elevatorState != ElevatorState.IDLE && elevatorState != ElevatorState.HOLDING) {
            return;
        }


        // Don't go beyond the top or we will break the strings
        if (lSpindle.getCurrentPosition() >= TOP || rSpindle.getCurrentPosition() >= TOP) {
            manualHoldElevator();
            return;
        }

        elevatorState = ElevatorState.MANUAL_UP;
        teamUtil.log("Moving Elevator UP slowly: ");
        teamUtil.log("SpindlePosition: " + lSpindle.getCurrentPosition() + ":" + rSpindle.getCurrentPosition());
        rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSpindle.setPower(.2);
        lSpindle.setPower(.2);
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Move down to a specific position quickly
    private void moveElevatorDown(int position, long timeOut) {
        if (!isSafeToElevate()) {
            teamUtil.log("ERROR: called moveElevator while not safe to elevate");
            return;
        }

        if (lSpindle.getCurrentPosition() < position || rSpindle.getCurrentPosition() < position) {
            teamUtil.log("ERROR: called moveElevatorDown with a target position above current position");
            return;
        }
        elevatorState = ElevatorState.MOVING_DOWN;
        teamUtil.log("Moving Elevator DOWN to: " + position);
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut = false;

        rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Go down fast
        int slowDownPoint = position + 100;
        rSpindle.setPower(-.5);
        lSpindle.setPower(-.5);
        while ((rSpindle.getCurrentPosition() > slowDownPoint) && (lSpindle.getCurrentPosition() > slowDownPoint) && teamUtil.keepGoing(timeOutTime)) {
            // wait for it to get close to the target
            teamUtil.log("Spindles: " + lSpindle.getCurrentPosition() + " : " + rSpindle.getCurrentPosition()); // uncomment for play by play...

        }
        // slow down for the last bit
        rSpindle.setPower(-.3);
        lSpindle.setPower(-.3);
        while ((rSpindle.getCurrentPosition() > position) && (lSpindle.getCurrentPosition() > position) && teamUtil.keepGoing(timeOutTime)) {
            // wait for it to get to the target
            teamUtil.log("Spindles: " + lSpindle.getCurrentPosition() + " : " + rSpindle.getCurrentPosition()); // uncomment for play by play...
        }

        // bounce back up if needed and hold position
        rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSpindle.setTargetPosition(position);
        lSpindle.setTargetPosition(position);
        rSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lSpindle.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rSpindle.setPower(.99);
        lSpindle.setPower(.99);

        elevatorState = ElevatorState.HOLDING;
        timedOut = (System.currentTimeMillis() > timeOutTime);
        if (timedOut) {
            teamUtil.log("Moving Elevator DOWN - TIMED OUT!");
            moveElevatorToBottomNoWait();
        } else {
            teamUtil.log("Moving Elevator DOWN - Finished");
        }
    }


    // start the elevator moving down slowly, presumably under manual control
    // This expects to be called over and over while a control is being pressed so
    // that we can stop if we reach the bottom!
    // TODO: after moving up, when you try to go back down, sometimes it jumps...RUN_TO_POSITION leftovers from holding?  Moved calls to adjust power before calls to change mode to see is that fixes it...still need to test
    public void moveElevatorDownSlowly() {
        if (!isSafeToElevate()) {
            teamUtil.log("ERROR: called moveElevatorDownSlowly while not safe to elevate");
            return;
        }
        if (elevatorState != ElevatorState.IDLE && elevatorState != ElevatorState.HOLDING) {
            return;
        }

        if (lSpindle.getCurrentPosition() <= BOTTOM || rSpindle.getCurrentPosition() <= BOTTOM) {
            rSpindle.setPower(0);
            lSpindle.setPower(0);
            elevatorState = ElevatorState.IDLE;
            return;
        }

        elevatorState = ElevatorState.MANUAL_DOWN;
        teamUtil.log("Moving Elevator DOWN slowly: ");
        teamUtil.log("SpindlePosition: " + lSpindle.getCurrentPosition() + ":" + rSpindle.getCurrentPosition());
        //TODO: switched the order of these subsequent commands
        rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rSpindle.setPower(-.1);
        lSpindle.setPower(-.1);

    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveElevator(int position, long timeOut) {
        if (lSpindle.getCurrentPosition() < position) {
            moveElevatorUp(position, timeOut);
        } else {
            moveElevatorDown(position, timeOut);
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveElevatorNoWait(final int position, final long timeOut) {
        if (elevatorState == ElevatorState.IDLE) {
            elevatorState = ElevatorState.MOVING_UP;
            teamUtil.log("Launching Thread to Move Elevator");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    moveElevator(position, timeOut);
                }
            });
            thread.start();
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Move the elevator up to the specified level
    public void moveElevatorToLevel(int level, long timeOut) {
        elevatorState = ElevatorState.MOVING_UP;
        if (level > MAX_LEVELS) {
            teamUtil.log("ERROR: called GoToLevel with level=" + level);
            level = MAX_LEVELS;
        }
        int target = LEVEL_0 + (LEVEL_INCREMENT * level);
        moveElevator(target, timeOut);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveElevatorToLevelNoWait(final int level, final long timeOut) {
        if (elevatorState == ElevatorState.IDLE) {
            elevatorState = ElevatorState.MOVING_UP;
            teamUtil.log("Launching Thread to Go To Level");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    moveElevatorToLevel(level, timeOut);
                }
            });
            thread.start();
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Drop the elevator to its lowest point
    private void moveElevatorToBottomActive() {
        teamUtil.log("started moveElevatorToBottomActive");
        rSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Go down fast
        rSpindle.setPower(-.5);
        lSpindle.setPower(-.5);
        while ((rSpindle.getCurrentPosition() > LEVEL_0) && (lSpindle.getCurrentPosition() > LEVEL_0)) {
            // wait for it to get close to bottom
        }
        // slow down for the last bit
        rSpindle.setPower(-.3);
        lSpindle.setPower(-.3);
        while ((rSpindle.getCurrentPosition() > BOTTOM) && (lSpindle.getCurrentPosition() > BOTTOM)) {
            // wait for it to get  to bottom
        }

        // Try to keep the motors from overroating on the drop so we don't have lots of loose string...
        rSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSpindle.setPower(.1);
        lSpindle.setPower(.1);
        teamUtil.sleep(100);
        rSpindle.setPower(0);
        lSpindle.setPower(0);
        elevatorState = ElevatorState.IDLE;
        teamUtil.log("activeGoToBottom - Finished");
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    private void moveElevatorToBottomPassive() {
        elevatorState = ElevatorState.MOVING_DOWN;
        rSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lSpindle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rSpindle.setPower(-0.1);
        lSpindle.setPower(-0.1);
        teamUtil.sleep(2500);
        elevatorState = ElevatorState.IDLE;
        teamUtil.log("Moving Elevator to Bottom - Finished");
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveElevatorToBottom() {
        teamUtil.log("Moving Elevator to Bottom");
        if (!isSafeToElevate()) {
            teamUtil.log("ERROR: called moveElevatorToBottom while not safe to elevate");
            return;
        }
        if (elevatorState != ElevatorState.IDLE) { // if we are not already at the bottom
            elevatorState = ElevatorState.MOVING_DOWN;
            teamUtil.log("firstSpindlePosition: " + rSpindle.getCurrentPosition());
            moveElevatorToBottomActive();  // OR moveElevatorToBottomPassive();
        } else {
            teamUtil.log("WARNING: called moveElevatorToBottom when it was already there");

        }
    }


    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void moveElevatorToBottomNoWait() {
        teamUtil.log("elevatorstate is " + elevatorState);
        if ((elevatorState == ElevatorState.HOLDING)) {
            elevatorState = ElevatorState.MOVING_DOWN;
            teamUtil.log("Launching Thread to Move Elevator to Bottom");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    moveElevatorToBottom();
                }
            });
            thread.start();
        }
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    public void liftTelemetry() {
        teamUtil.telemetry.addData("liftState:", liftState);
        teamUtil.telemetry.addData("elevator State:", elevatorState);
        if (liftBaseIsDown()) {
            teamUtil.telemetry.addLine("Lift Base is DOWN");
        } else if (liftBaseIsUp()) {
            teamUtil.telemetry.addLine("Lift Base is UP");
        } else {
            teamUtil.telemetry.addLine("Lift Base lift position UNKNOWN");
        }
        teamUtil.telemetry.addData("Lift Motor Power:", liftBasePower);
        teamUtil.telemetry.addData("Lift Base Encoder:", liftBase.getCurrentPosition());
        teamUtil.telemetry.addData("liftDownLimit:", liftDownLimit.isPressed());
        teamUtil.telemetry.addData("liftUpMag1:", !liftUpMagSwitch1.getState());
        teamUtil.telemetry.addData("liftUpMag2:", !liftUpMagSwitch2.getState());
        teamUtil.telemetry.addData("lSpindle:", lSpindle.getCurrentPosition());
        teamUtil.telemetry.addData("rSpindle:", rSpindle.getCurrentPosition());
        teamUtil.telemetry.addData("spindlesCalibrated:", spindlesCalibrated);
        teamUtil.telemetry.addData("baseCalibrated:", baseCalibrated);
    }


    /////////////////////////////////////////////////////////////////////////
    // Some stuff for testing only

    // Keep track of when controls are updated to limit how fast values can change
    private long nextControlUpdate = System.currentTimeMillis();
    private final long CONTROL_INTERVAL = 500;
    private double liftBasePower = .5;


    public void increaseLiftBasePower() {
        // If enough time has passed since we last updated the power
        if (System.currentTimeMillis() > nextControlUpdate) {
            liftBasePower = liftBasePower + 0.1;
            nextControlUpdate = System.currentTimeMillis() + CONTROL_INTERVAL;
        }
    }

    public void decreaseLiftBasePower() {
        // If enough time has passed since we last updated the power
        if (System.currentTimeMillis() > nextControlUpdate) {
            liftBasePower = liftBasePower - 0.1;
            nextControlUpdate = System.currentTimeMillis() + CONTROL_INTERVAL;
        }
    }

    public void liftBaseUp() {
        if (liftBase.getCurrentPosition() < LIFT_BASE_TOP_LIMIT_ENCODER_CLICKS) {
            liftBase.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftBase.setPower(liftBasePower);
        }
    }

}



