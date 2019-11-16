package org.firstinspires.ftc.teamcode.basicLibs;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

public class teamUtil {

    public static final double YELLOW_JACKET_ENCODER_CLICKS=145.6;
    public static final double NEVERREST40_ENCODER_CLICKS=1120;
    public static LinearOpMode theOpMode;
    public static final double NEVEREST20_ENCODER_CLICKS=537.6;

    /**
     * This method puts the current thread to sleep for the given time in msec.
     * It handles InterruptException where it recalculates the remaining time
     * and calls sleep again repeatedly until the specified sleep time has past.
     *
     * @param sleepTime specifies sleep time in msec.
     */
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

    public static void log(String logString) {
        RobotLog.d("14140LOG: " + logString);
    }

}
