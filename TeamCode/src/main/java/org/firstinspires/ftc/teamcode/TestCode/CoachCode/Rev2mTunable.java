package org.firstinspires.ftc.teamcode.TestCode.CoachCode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.RobotLog;

import static com.qualcomm.hardware.stmicroelectronics.VL53L0X.Register.FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT;

public class Rev2mTunable extends Rev2mDistanceSensor {
    public Rev2mTunable(I2cDeviceSynch deviceClient) {
        super(deviceClient);
    }
    @Override
    public String getDeviceName() {
        return "REV 2M Tunable Sensor";
    }

    @Override
    protected synchronized boolean doInitialize(){
        boolean result = super.doInitialize();
        adjustSensor();
        return result;
    }

    protected synchronized void adjustSensor() {
        // debug.
        // check rate limit before we set it.
        RobotLog.dd(MYTAG, "Tunable Sensor:initial sig rate lim (MCPS) %.06f", ourGetSignalRateLimit());

        // set new range signal rate limit to 0.25 MCPS (million counts per second)
        ourSetSignalRateLimit((float)1.0);

        // debug.
        // check rate limit after we set it.
        RobotLog.dd(MYTAG, "Tunable Sensor:adjusted sig rate lim (MCPS) %.06f", ourGetSignalRateLimit());

    }

    private boolean ourSetSignalRateLimit(float limit_Mcps)
    {
        // check range.
        if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

        // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
        writeShort(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (short)(limit_Mcps * (1 << 7)));
        return true;
    }
    private float ourGetSignalRateLimit()
    {
        return (float)readShort(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
    }


}
