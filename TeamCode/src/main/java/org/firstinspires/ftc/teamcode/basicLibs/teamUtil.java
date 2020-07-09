package org.firstinspires.ftc.teamcode.basicLibs;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Assemblies.Robot;

import java.util.LinkedList;
import java.util.Queue;

public class teamUtil {

    public static final double YELLOW_JACKET_ENCODER_CLICKS=145.6;
    public static final double NEVERREST40_ENCODER_CLICKS=1120;
    public static final double NEVEREST20_ENCODER_CLICKS=537.6;

    public static LinearOpMode theOpMode;
    public static boolean inInitialization = true;
    public static Blinkin theBlinkin =null;
    private static Robot robot;
    public static Telemetry telemetry;
    public enum Alliance {RED, BLUE}

    public static Alliance alliance = Alliance.RED;


    /**
     * This method puts the current thread to sleep for the given time in msec.
     * It handles InterruptException where it recalculates the remaining time
     * and calls sleep again repeatedly until the specified sleep time has past.
     *
//     * @param sleepTime specifies sleep time in msec.
//     */

    public static void init (LinearOpMode opMode) {
        theOpMode = opMode;
        telemetry = theOpMode.telemetry;
        inInitialization = true;
        //theBlinkin = new Blinkin(opMode.hardwareMap, opMode.telemetry);
        //theBlinkin.init();
    }
    public static void sleep(long sleepTime) {
        long wakeupTime= System.currentTimeMillis()+sleepTime;
        while(sleepTime>0){
            try {
                Thread.sleep(sleepTime);
            } catch(InterruptedException e) {
            }
            sleepTime=wakeupTime- System.currentTimeMillis();
        }
    }   //sleep

    public static void pause(long sleepTime) {
        long wakeupTime= System.currentTimeMillis()+sleepTime;
        while(System.currentTimeMillis()< wakeupTime){
        }
    }


//    public static Robot getRobot(){
//        if(robot == null){
//            robot = new Robot(theOpMode);
//            teamUtil.log("created new robot");
//        } else{
//            teamUtil.log("recycling robot");        }
//
//        teamUtil.log("here's a robot");
//        return robot;
//    }


//    public static void nukeRobot(){
//        robot = null;
//    }


    // log something so we can filter out the FTC robot log info
    // 3 -> 0 = getStackTrace, 1 = teamUtil.log, 3 = whoever called us
    public static void log(String logString) {
        RobotLog.d("14140LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    public static boolean keepGoing(long timeOutTime) {
        return ((inInitialization || theOpMode.opModeIsActive()) && (System.currentTimeMillis() < timeOutTime));
    }



    // trackPerf : Call this in liftLoop() in your opmode to LOGCAT some stats on your software performance
    static sampleStats samples;
    static long lastLoopCall = Long.MAX_VALUE;

    public static void initPerf() {
        teamUtil.log("PERF---Initializing");
        samples = new sampleStats();
        samples.setTimeWindow(3000);
        lastLoopCall = Long.MAX_VALUE;
    }
    public static void trackPerf() {
        long now = System.currentTimeMillis();
        if (lastLoopCall == Long.MAX_VALUE) { // first time, initialize everything
            lastLoopCall = now;
        } else {
            samples.addSample((double) (now - lastLoopCall));
            lastLoopCall = now;
            teamUtil.log("PERF---RA:" + samples.getRunningAverage() + " L:" + samples.getLow() + " H:" + samples.getHigh());
        }
    }

}
