package org.firstinspires.ftc.teamcode.basicLibs;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import java.util.LinkedList;
import java.util.Queue;

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

    // sampleStats : A utility class for keeping track of some useful stats on a sample stream
    // currently supports a running average over the last N samples OR the last M milliseconds
    // Also tracks low/high/range over the entire sample stream
    // It would be good to limit the low/high/range to the running sample window but I can't figure
    // out how to do that right now in Time O(1)
    public class sampleStats{
        private boolean timeMode = false; // can't use an enum in a nested class... :(
        private double total;
        private double low;
        private double high;
        private int numSamplesInWindow;
        private long timeWindow;
        private class sampleWrapper {
            public double sample;
            public long sampleTime;
            sampleWrapper(double s, long t){
                sample = s;
                sampleTime = t;
            }
        }
        private LinkedList<sampleWrapper> sampleQueue = new LinkedList<>();



        public void setTimeWindow (int milliSeconds) {
            timeMode = true;
            timeWindow = milliSeconds;
            sampleQueue.clear();
            low = Double.MAX_VALUE;
            high = -Double.MAX_VALUE;
        }
        public void setSampleNumberWindow (int num) {
            timeMode = false;
            numSamplesInWindow = num;
            sampleQueue.clear();
            low = Double.MAX_VALUE;
            high = -Double.MAX_VALUE;
        }
        public void addSample (double sample) {
            if (timeMode) { // keep a running list of samples (and their total) that have been taken in the last timeWindow milliseconds
                long now = System.currentTimeMillis();
                long cutoff = now - timeWindow;
                total += sample;
                sampleQueue.add(new sampleWrapper(sample, now));
                while (sampleQueue.getLast().sampleTime<cutoff) {
                    total -= sampleQueue.removeLast().sample;
                }
            } else { // keep a running list of the last numSamplesInWindow Samples (and their total)
                total += sample;
                sampleQueue.add(new sampleWrapper(sample, System.currentTimeMillis()));
                if (sampleQueue.size() > numSamplesInWindow) {
                    total -= sampleQueue.remove().sample; // this would be more efficient if we re-used the object being removed for the one being added...
                }
            }
            if (sample < low) {
                low = sample;
            }
            if (sample > high) {
                high = sample;
            }
        }

        public double getRunningAverage(){
            return total / sampleQueue.size();
        }
        public double getLow(){
            return low;
        }
        public double getHigh(){
            return high;
        }
        public double getRange(){
            return (high-low);
        }
    }
}
