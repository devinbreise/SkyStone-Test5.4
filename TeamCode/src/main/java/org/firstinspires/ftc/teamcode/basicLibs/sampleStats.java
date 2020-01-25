package org.firstinspires.ftc.teamcode.basicLibs;

import java.util.LinkedList;

// sampleStats : A utility class for keeping track of some useful stats on a sample stream
// currently supports a running average over the last N samples OR the last M milliseconds
// Also tracks low/high/range over the entire sample stream
// It would be good to limit the low/high/range to the running sample window but I can't figure
// out how to do that right now in Time O(1)
public class sampleStats {
    enum sampleMode {TIME, SAMPLE_SIZE}
    private sampleMode mode = sampleMode.SAMPLE_SIZE;
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
        mode = sampleMode.TIME;
        timeWindow = milliSeconds;
        sampleQueue.clear();
        low = Double.MAX_VALUE;
        high = -Double.MAX_VALUE;
    }
    public void setSampleNumberWindow (int num) {
        mode = sampleMode.SAMPLE_SIZE;
        numSamplesInWindow = num;
        sampleQueue.clear();
        low = Double.MAX_VALUE;
        high = -Double.MAX_VALUE;
    }
    public void addSample (double sample) {
        if (mode == sampleMode.TIME) { // keep a running list of samples (and their total) that have been taken in the last timeWindow milliseconds
            long now = System.currentTimeMillis();
            long cutoff = now - timeWindow;
            total += sample;
            sampleQueue.addFirst(new sampleWrapper(sample, now));
            while (sampleQueue.getLast().sampleTime<cutoff) {
                total -= sampleQueue.removeLast().sample;
            }
        } else { // keep a running list of the last numSamplesInWindow Samples (and their total)
            total += sample;
            sampleQueue.addFirst(new sampleWrapper(sample, System.currentTimeMillis()));
            if (sampleQueue.size() > numSamplesInWindow) {
                total -= sampleQueue.getLast().sample; // this would be more efficient if we re-used the object being removed for the one being added...
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

