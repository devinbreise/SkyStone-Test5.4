package org.firstinspires.ftc.teamcode.Assemblies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.basicLibs.teamUtil;

public class Lift {

    private double liftBasePower = .5;
    private DcMotor liftBase;

    // Keep track of when controls are updated to limit how fast values can change
    private long nextControlUpdate = System.currentTimeMillis();
    private final long CONTROL_INTERVAL = 500;

    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Lift(HardwareMap theHardwareMap, Telemetry theTelmetry){
        hardwareMap = theHardwareMap;
        telemetry = theTelmetry;

    }

    public void initLift(){
        liftBase = hardwareMap.dcMotor.get("liftBase");
    }

    public void liftUp(){
        liftBase.setPower(liftBasePower);
    }

    public void liftDown(){
        liftBase.setPower(-liftBasePower);
    }

    public void shutDownLift(){
        liftBase.setPower(0);
    }

    public void increaseLiftPower(){
        // If enough time has passed since we last updated the power
        if (System.currentTimeMillis() > nextControlUpdate)
        {
            liftBasePower = liftBasePower + 0.1;
            nextControlUpdate = System.currentTimeMillis() + CONTROL_INTERVAL;
        }
    }
    public void decreaseLiftPower(){
        // If enough time has passed since we last updated the power
        if (System.currentTimeMillis() > nextControlUpdate)
        {
            liftBasePower = liftBasePower - 0.1;
            nextControlUpdate = System.currentTimeMillis() + CONTROL_INTERVAL;
        }
    }

    public void liftTelemetry(){
        telemetry.addData("Lift Motor Power:", liftBasePower);
    }
}
